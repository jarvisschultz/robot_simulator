#!/usr/bin/env python
"""
Jarvis Schultz
July 2012

This program is a quick first attempt at integrating trep with ROS.  I
will define a trep system that is basically a suspended mass hanging
from a controllable point in 3D space.  The length of the "string"
connected is also controllable. There is one callback that subscribes
to a message that contains the pose of the robot, and the lengths of
both strings.  For now, I just assume that the left string is the one
controlling the length of the string.

Everytime the odometry callback is called, we use trep to step the
variational integrator forward in time, and we then publish the
location of the dynamic variables as a PointStamped.
"""

## define all imports:
import roslib; roslib.load_manifest('robot_simulator')
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import Point
from puppeteer_msgs.msg import FullRobotState
from puppeteer_msgs.msg import PlanarSystemConfig
import trep
from trep import tx, ty, tz, rx, ry, rz
from math import sin, cos
from math import pi as mpi
import numpy as np
import sys
import scipy as sp

## define some global constants:
BALL_MASS = 0.1244 ## kg
g = 9.81 ## m/s^2
h0 = 1 ## default height of robot in m
DT = 1/30.0 ## nominal dt for the system


## define some miscellaneous geometry helper functions
def to_se3(R,p):
    """
    This function takes in a vector and a rotation matrix, and returns
    an element of SE(3) as a 4x4 numpy array
    """
    pt = np.ravel(p)
    if np.size(pt) != 3:
        print "Wrong size vector when converting to SE(3)"
        return 1
    elif R.shape != (3,3):
        print "Wrong size rotation matrix when converting to SE(3)"
        return 1
    else:
        return np.vstack((np.hstack((R,np.array([pt]).T)),np.array([0,0,0,1])))


def hat(w):
    """
    This function implements the 3D 'hat' operator
    """
    wt = np.ravel(w)
    return np.array([[0,-wt[2],wt[1]],
            [wt[2],0,-wt[0]],
            [-wt[1],wt[0],0]])

def quat_to_rot(quat):
    """
    converts a quaternion of form [w,x,y,z] to a 3x3 element of
    SO(3) (represented as a numpy array)
    """
    quat = np.array(quat)
    q = np.ravel(quat)
    if np.size(q) != 4:
        print "Invalid quaternion passed to quat_to_rot()"
        return 1
    th = 2*np.arccos(q[0])
    if th == 0:
        w = np.array([0,0,0])
    else:
        w = q[1:]/np.sin(th/2.0)
    R = np.eye(3)+hat(w)*sin(th)+np.dot(hat(w),hat(w))*(1-cos(th))
    return R



## define a class for simulating the system.
class MassSystem3D:
    """
    Class for integrating system.  Includes many helper functions.
    """
    def __init__(self, mass=BALL_MASS, q0=None):
        self.mass = mass
        self.q0 = q0
        self.sys = self.create_system()
        ## set initial configuration variables
        if q0:
            if len(q0) < sys.nQ:
                print "Invalid number of initial conditions"
                sys.exit()
            self.sys.q = {
                'xm' : q0[0],
                'ym' : q0[1],
                'zm' : q0[2],
                'xr' : q0[3],
                'zr' : q0[4],
                'r' : q0[5],
                }

        self.sys.satisfy_constraints()
        self.mvi = trep.MidpointVI(self.sys)
        self.mvi.initialize_from_configs(0,self.sys.q,DT,self.sys.q)

        return

    def get_current_configuration(self):
        return self.mvi.q2

    def get_current_time(self):
        return self.mvi.t2

    def reset_integration(self, state=None):
        if state:
            self.q0 = state
        if self.q0:
            self.sys.q = {
                'xm' : self.q0[0],
                'ym' : self.q0[1],
                'zm' : self.q0[2],
                'xr' : self.q0[3],
                'zr' : self.q0[4],
                'r' : self.q0[5],
                }
        self.sys.satisfy_constraints()
        self.mvi.initialize_from_configs(0,self.q0,DT,self.q0)

        return

    def create_system(self):
        # define system:
        system = trep.System()

        frames = [
            tx('xm', name='x-mass'), [
                ty('ym', name='y-mass'), [
                    tz('zm', name='z-mass', mass=self.mass) ]],
            ty(h0, name='robot_plane'), [
                tx('xr', name='x-robot', kinematic=True), [
                    tz('zr', name='z-robot', kinematic=True) ]]]
        system.import_frames(frames)
        trep.potentials.Gravity(system, (0, -g, 0))
        trep.forces.Damping(system, 0.05)

        # add string constraint as a kinematic configuration var
        trep.constraints.Distance(system, 'z-mass', 'z-robot','r')

        return system


    def take_step(self, dt=DT, rho=()):
        self.mvi.step(self.mvi.t2+dt, (), rho)
        return



## Now let's define a class for ros to use.  We will define all
## subscriptions, publishers, and callbacks
class MassSimulator:
    def __init__(self):
        rospy.loginfo("Starting mass_simulator node!")

        ## define a subscriber and callback for the robot_simulator
        self.sub = rospy.Subscriber("robot_state", FullRobotState, self.inputcb)

        ## define a publisher for the position of the mass:
        self.mass_pub = rospy.Publisher("mass_location", PointStamped)

        ## define a publisher for publishing the 2D results of the
        ## simulation
        self.plan_pub = rospy.Publisher("meas_config", PlanarSystemConfig)

        ## define a transform listener for publishing the transforms
        ## to the location of the mass
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        ## define the system that we will use to integrate the dynamics
        self.sys = MassSystem3D()
        self.initialized_flag = False

        ## set base time
        self.last_time = rospy.rostime.get_rostime()
        self.len = h0

        ## get noise value:
        if rospy.has_param("/simulator_noise"):
            self.noise = rospy.get_param("/simulator_noise")
        else:
            self.noise = 0.0

        return

    def inputcb(self, data):
        rospy.logdebug("inputcb triggered")

        # translation from base_link to left string expressed in the
        # base_link frame:
        vb = np.array([[0.0102, 0.0391, 0.086, 0]]).T
        gbs = np.array([[1, 0, 0, vb[0,0]],
                       [0, 0, -1, vb[1,0]],
                       [0, 1, 0, vb[2,0]],
                       [0, 0, 0, 1]])
        gsb = np.linalg.inv(gbs)

        # map to base stuff:
        pbm = np.array([[data.pose.pose.position.x,
                         data.pose.pose.position.y,
                         data.pose.pose.position.z]]).T
        qbm = np.array([[data.pose.pose.orientation.w,
                         data.pose.pose.orientation.x,
                         data.pose.pose.orientation.y,
                         data.pose.pose.orientation.z]]).T
        Rbm = quat_to_rot(qbm)
        gbm = to_se3(Rbm,pbm)
        gmb = np.linalg.inv(gbm)
        vm = np.dot(gmb,np.dot(gbs,np.dot(gsb,vb)))
        vm = np.ravel(vm)[0:3]

        ## so the first thing that we need to do is get the location
        ## of the robot in the "/optimization_frame"
        p = PointStamped()
        p.header = data.pose.header
        p.point = data.pose.pose.position
        p.point.x -= vm[0]
        p.point.y -= vm[1]
        p.point.z -= vm[2]
        quat = QuaternionStamped()
        quat.quaternion = data.pose.pose.orientation
        quat.header = p.header
        try:
            ptrans = self.listener.transformPoint("/optimization_frame", p)
            qtrans = self.listener.transformQuaternion("/optimization_frame", quat)
        except (tf.Exception):
            rospy.loginfo("tf exception caught !")
            return

        ## if we have not initialized the VI, let's do it, otherwise,
        ## let's integrate
        if not self.initialized_flag:
            q = [
                ptrans.point.x,
                ptrans.point.y-h0,
                ptrans.point.z,
                ptrans.point.x,
                ptrans.point.z,
                h0 ]
            self.sys.reset_integration(state=q)
            self.last_time = ptrans.header.stamp
            self.initialized_flag = True
            return

        # get operating_condition:
        if rospy.has_param("/operating_condition"):
            operating = rospy.get_param("/operating_condition")
        else:
            return
            

        # set string length
        self.len = data.left;

        ## if we are not running, just reset the parameter:
        if operating is not 2:
            self.initialized_flag = False
            return
        else:
            self.initialized_flag = True
            # if we are in the "running mode", let's integrate the VI,
            # and publish the results:
            rho = [ptrans.point.x, ptrans.point.z, self.len]
            dt = (ptrans.header.stamp - self.last_time).to_sec()
            self.last_time = ptrans.header.stamp
            rospy.logdebug("Taking a step! dt = "+str(dt))
            self.sys.take_step(dt, rho)
            ## now we can get the state of the system
            q = self.sys.get_current_configuration()

            ## now add the appropriate amount of noise:
            q[0] += self.noise*np.random.normal()
            new_point = PointStamped()
            new_point.point.x = q[0]
            new_point.point.y = q[1]
            new_point.point.z = q[2]
            new_point.header.frame_id = "/optimization_frame"
            new_point.header.stamp = rospy.rostime.get_rostime()
            rospy.logdebug("Publishing mass location")
            self.mass_pub.publish(new_point)

            ## we can also publish the planar results:
            config = PlanarSystemConfig()
            config.header.frame_id = "/optimization_frame"
            config.header.stamp = rospy.get_rostime()
            config.xm = q[0]
            config.ym = q[1]
            config.xr = rho[0]
            config.r = rho[2]
            self.plan_pub.publish(config)

            ## now we can send out the transform
            ns = rospy.get_namespace()
            fr = "mass_location"
            if len(ns) > 1:
                fr = rospy.names.canonicalize_name(ns+fr)

            # here we are going to do a bunch of geometry math to
            # ensure that the orientation of the frame that we are
            # placing at the mass location looks "nice"

            # zvec points from robot to the string
            zvec = np.array([q[0]-ptrans.point.x, q[1]-ptrans.point.y, q[2]-ptrans.point.z])
            zvec = zvec/np.linalg.norm(zvec)
            # get transform from incoming header frame to the optimization frame
            quat = qtrans.quaternion
            qtmp = np.array([quat.x, quat.y, quat.z, quat.w])
            # convert to SO(3), and extract the y-vector for the frame
            R1 = tf.transformations.quaternion_matrix(qtmp)[:3,:3]
            yvec = -R1[:,1]
            yvec[1] = (-yvec[0]*zvec[0]-yvec[2]*zvec[2])/zvec[1]
            # x vector is a cross of y and z
            xvec = np.cross(yvec, zvec)
            # build rotation matrix and send transform
            R = np.column_stack((xvec,yvec,zvec,np.array([0,0,0])))
            R = np.row_stack((R,np.array([0,0,0,1])))
            quat = tuple(tf.transformations.quaternion_from_matrix(R).tolist())
            self.br.sendTransform((new_point.point.x, new_point.point.y, new_point.point.z),
                                  quat,
                                  new_point.header.stamp,
                                  fr,
                                  "/optimization_frame")



        return



def main():
    """
    Run the main loop, by instatiating a MassSimulator, and then
    calling ros.spin
    """
    rospy.init_node('mass_simulator', log_level=rospy.INFO)

    try:
        sim = MassSimulator()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
