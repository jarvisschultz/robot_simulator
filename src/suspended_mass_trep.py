## define all imports:
import rospy
import roslib; roslib.load_manifest('robot_simulator')
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
BALL_MASS = 0.1244 # kg
g = 9.81 # m/s^2
h0 = 1 # default height of robot in m
DT = 1/30.0 # nominal dt for the system

## define a class for simulating the system.
class 3DMassSystem:
    """
    Class for integrating system.  Includes many helper functions.
    """
    def __init__(self, mass=BALL_MASS, q0=None):
        sys = self.create_system()
        # set initial configuration variables
        if q0:
            if len(q0) < sys.nQ:
                print "Invalid number of initial conditions"
                sys.exit()
            sys.q = {
                'xm' : q0[0],
                'ym' : q0[1],
                'zm' : q0[2],
                'xr' : q0[3],
                'zr' : q0[4],
                'r' : q0[5],
                }

        system.satisfy_constraints()
        mvi = trep.MidpointVI(sys)
        mvi.initialize_from_configs(0,q0,DT,q0)

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
        self.sys.satisfy_constants()
        del self.mvi
        self.mvi = trep.MidpointVI(sys)
        self.mvi.initialize_from_configs(0,q0,DT,q0)

        return


    def take_step(self, dt=DT, rho=()):
        self.mvi.step(self.mvi.t2+dt, (), rho)
        return
    
        

## Now let's define a class for ros to use.  We will define all
## subscriptions, publishers, and callbacks
class MassSimulator:
    def __init__(self):
        ## define a subscriber and callback for the robot_simulator
        rospy.Subscriber("vo_noise_free", Odometry, self.inputcb)
        
        ## define a publisher for the position of the mass:
        rospy.Publisher("mass_location", PointStamped)

        ## define a transform listener for publishing the transforms
        ## to the location of the mass
        br = tf.TransformBroadcaster()
        listener = tf.TransformListener()

        ## define the system that we will use to integrate the dynamics
        sys = 3DMassSystem()
        initialized_flag = False

        ## set base time
        last_time = rospy.rostime.get_rostime()

        return
    
                
    def inputcb(self, data):
        ## so the first thing that we need to do is get the location
        ## of the robot in the "/optimization_frame"
        p = PointStamped()
        p.header = data.header
        p.point = data.pose.pose.position
        try:
            ptrans = listener.transformPoint("/optimization_frame", p)
        except: (tf.Exception):
            rospy.logwarn("tf exception caught !")
            return

        ## if we have not initialized the VI, let's do it, otherwise,
        ## let's integrate
        if not initialized_flag:
            q = [
                ptrans.x,
                ptrans.y-h0,
                ptrans.z,
                ptrans.x,
                ptrans.z,
                h0 ]                
            self.sys.reset_integration(state=q)
            self.last_time = ptrans.header.stamp 
            return

        ## if we are in the "running mode", let's integrate the VI,
        ## and publish the results:
        rho = [ptrans.x, ptrans.z, h0]
        dt = rospy.rostime.get_time(ptrans.header.stamp - self.last_time)
        
        self.sys.take_step(dt, rho)

        



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
        
        
                
        
        
