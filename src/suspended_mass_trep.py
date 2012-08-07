#!/usr/bin/env python
"""
Jarvis Schultz
July 2012

This program is a quick first attempt at integrating trep with ROS.  I
will define a trep system that is basically a suspended mass hanging
from a controllable point in 3D space.  The length of the "string"
connected is also controllable.  I have one callback that subsribes to
a nav_msgs::Odometry message that indicates the location of the
kinematic variables 'xr' and 'zr'.  I use an odometry message because
I use a simulator of a differential drive robot to actually publish
this topic.  I have another callback that subscribes to a
geometry_msgs::Point that sets the length of the string; 'r'.  In my
application the same robot simulator also publishes this topic.

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
        del self.mvi
        self.mvi = trep.MidpointVI(self.sys)
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

        return

    def inputcb(self, data):
        rospy.logdebug("inputcb triggered")

        ## let's try and get the transform from the
        ## base_footprint_kinect frame out to the string position
        try:
            (off, rot) = self.listener.lookupTransform("/robot_1/base_link",
                                                       "/robot_1/left_string",
                                                       rospy.Time())
            tmp = PointStamped()
            tmp.header = data.pose.header
            tmp.header.stamp = self.listener.getLatestCommonTime("/robot_1/base_link",
                                                                 "/optimization_frame")
            tmp.header.frame_id = "/robot_1/base_link"
            tmp.point.x = off[0]
            tmp.point.y = off[1]
            tmp.point.z = off[2]
            offtrans = self.listener.transformPoint("/optimization_frame", tmp)
        except (tf.Exception):
            rospy.loginfo("tf could not get string offset!")
            return

        ## so the first thing that we need to do is get the location
        ## of the robot in the "/optimization_frame"
        p = PointStamped()
        p.header = data.pose.header
        p.point = data.pose.pose.position
        # p.point.x -= offtrans[0]
        # p.point.y -= offtrans[1]
        # p.point.z -= offtrans[2]
        quat = QuaternionStamped()
        quat.quaternion = data.pose.pose.orientation
        quat.header = p.header
        try:
            ptrans = self.listener.transformPoint("/optimization_frame", p)
            qtrans = self.listener.transformQuaternion("/optimization_frame", quat)
        except (tf.Exception):
            rospy.loginfo("tf exception caught !")
            return
        ptrans.point.x -= offtrans.point.x
        ptrans.point.y -= offtrans.point.y
        ptrans.point.z -= offtrans.point.z

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

        ## get operating_condition:
        if rospy.has_param("/operating_condition"):
            operating = rospy.get_param("/operating_condition")
        else:
            return

        # set string length
        self.len = data.left;

        ## if we are not running, just reset the parameter:
        if operating in [0,1,3,4]:
            self.initialized_flag = False
            return
        else:
            self.initialized_flag = True
            ## if we are in the "running mode", let's integrate the VI,
            ## and publish the results:
            rho = [ptrans.point.x, ptrans.point.z, self.len]
            dt = (ptrans.header.stamp - self.last_time).to_sec()
            self.last_time = ptrans.header.stamp
            rospy.logdebug("Taking a step! dt = "+str(dt))
            self.sys.take_step(dt, rho)
            ## now we can get the state of the system
            q = self.sys.get_current_configuration()
            new_point = PointStamped()
            new_point.point.x = q[0]
            new_point.point.y = q[1]
            new_point.point.z = q[2]
            new_point.header.frame_id = "/optimization_frame"
            new_point.header.stamp = rospy.rostime.get_rostime()
            rospy.logdebug("Publishing mass location")
            self.mass_pub.publish(new_point)

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
