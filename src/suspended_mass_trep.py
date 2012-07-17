#!/usr/bin/env python
## define all imports:
import roslib; roslib.load_manifest('robot_simulator')
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
import trep
from trep import tx, ty, tz, rx, ry, rz
from math import sin, cos
from math import pi as mpi
import numpy as np
import sys
import tf
import math

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
        self.sub = rospy.Subscriber("vo_noise_free", Odometry, self.inputcb)
        
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

        return
    
                
    def inputcb(self, data):
        rospy.logdebug("inputcb triggered")
        ## so the first thing that we need to do is get the location
        ## of the robot in the "/optimization_frame"
        p = PointStamped()
        p.header = data.header
        p.point = data.pose.pose.position
        try:
            ptrans = self.listener.transformPoint("/optimization_frame", p)
        except (tf.Exception):
            rospy.logwarn("tf exception caught !")
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

        ## get operating_condition:
        operating = rospy.get_param("/operating_condition")

        ## if we are not running, just reset the parameter:
        if operating in [0,1,3,4]:
            self.initialized_flag = False
            return
        else:
            self.initialized_flag = True
            ## if we are in the "running mode", let's integrate the VI,
            ## and publish the results:
            rho = [ptrans.point.x, ptrans.point.z, h0]
            dt = (ptrans.header.stamp - self.last_time).to_sec()
            self.last_time = ptrans.header.stamp
            rospy.logdebug("Taking a step! dt = "+str(dt))
            self.sys.take_step(dt, rho)
            ## now we can get the state of the system
            q = self.sys.get_current_configuration()
            np = PointStamped()
            np.point.x = q[0]
            np.point.y = q[1]
            np.point.z = q[2]
            np.header.frame_id = "/optimization_frame"
            np.header.stamp = rospy.rostime.get_rostime()
            rospy.logdebug("Publishing mass location")
            self.mass_pub.publish(np)

            ## now we can send out the transform
            self.br.sendTransform((np.point.x, np.point.y, np.point.z),
                                  tf.transformations.quaternion_from_euler(0,0,0),
                                  np.header.stamp,
                                  "mass_location",
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
        
        
                
        
        
