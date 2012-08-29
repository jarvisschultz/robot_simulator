#!/usr/bin/env python
"""
Jarvis Schultz
August 2012

This program implements the full blown closed-loop controller for the
planar suspended mass system.  It must read in a trajectory mat file
from trep.  From this file we can get reference trajectories,
linearizations, and controller gains. This node subscribes to a
special message that contains all of the state variables for the
planar system.  This can be a noisy measurement, or an output from a
simulator.  This node then implements an EKF and publishes a filtered
version of the measured (or simulated signal).  To do this, it must
also subscribe to the inputs that have been sent to the robots (or
just use the ones that it also sends to the serial node or the
simulator). Once it has obtained the filtered state, it runs the
control law, and publishes commands for the simulator.

SUBSCRIPTIONS:
    - PlanarSystemConfig (meas_config)

PUBLISHERS:
    - PlanarSystemConfig (filt_config)
    - RobotCommands     (serial_commands)

NOTES:
    - Let's assume that the configurations that are received in
    this node are guaranteed to be in the right coordinate system.

"""

## define all imports:
import roslib; roslib.load_manifest('robot_simulator')
import rospy
import tf
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PointStamped
# from geometry_msgs.msg import QuaternionStamped
# from geometry_msgs.msg import Point
from puppeteer_msgs.msg import FullRobotState
from puppeteer_msgs.msg import PlanarSystemConfig
from puppeteer_msgs.msg import RobotCommands
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
class MassSystem2D:
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



class Filter:
    """
    This class is the base class for my EKF.  It must contain methods
    for instantiating a filter, modifying covariances, updating the
    filter, and returning any of the private filter parameters.
    """
    def __init__(self):

        return




class System:
    """
    This class will be responsible for creating an instance of the
    system, an instance of the filter, and reading in the results of
    an optimization to obtain controller gains and reference
    trajectories. It will also define all publishers and subscribers,
    and contain any callbacks.
    """
    def __init__(self):

        return



def main():
    """
    Run the main loop, by instatiating a Controller class, and then
    calling ros.spin
    """
    rospy.init_node('cl_control', log_level=rospy.INFO)

    try:
        sim = System()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
