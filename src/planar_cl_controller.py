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
from puppeteer_msgs.msg import FullRobotState
from puppeteer_msgs.msg import PlanarSystemConfig
from puppeteer_msgs.msg import RobotCommands
import trep
from trep import tx, ty, tz, rx, ry, rz
import trep.discopt as discopt
from math import sin, cos
from math import pi as mpi
import numpy as np
import sys
import scipy as sp
import os


## define some global constants:
BALL_MASS = 0.1244 ## kg
g = 9.81 ## m/s^2
h0 = 1 ## default height of robot in m
DT = 1/30.0 ## nominal dt for the system


# define a simple helper function for multiplying numpy arrays as
# matrices
def matmult(*x):
  """
  Shortcut for standard matrix multiplication.
  matmult(A,B,C) returns A*B*C.
  """
  return reduce(np.dot, x)


## define a class for simulating the system.
class MassSystem2D:
    """
    Class for integrating system.  Includes many helper functions.
    """
    def __init__(self, mass=BALL_MASS, q0=None, t=None):
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
                'xr' : q0[2],
                'r' : q0[3],
                }

        self.sys.satisfy_constraints()
        self.mvi = trep.MidpointVI(self.sys)
        return

    def get_current_configuration(self):
        return self.mvi.q2

    def get_current_time(self):
        return self.mvi.t2

    # the following function is just for creating the DSystem object
    # that we will use for integrating the system.
    def create_dsys(self, t):
        # now we can create a DSystem object:
        self.dsys = discopt.DSystem(self.mvi, t)
        return

    def create_system(self):
        # define system:
        system = trep.System()

        frames = [
            tx('xm', name='x-mass'), [
                ty('ym', name='y-mass', mass=BALL_MASS) ],
            ty(1, name='robot_plane'), [
                tx('xr', name='x-robot', kinematic=True) ]]
        system.import_frames(frames)
        trep.potentials.Gravity(system, (0, -g, 0))
        trep.forces.Damping(system, 0.05)

        # add string constraint as a kinematic configuration var
        trep.constraints.Distance(system, 'y-mass', 'x-robot','r')

        return system

    # def take_step(self, dt=DT, rho=()):
    #     self.mvi.step(self.mvi.t2+dt, (), rho)
    #     return

    # def reset_integration(self, state=None):
    #     if state:
    #         self.q0 = state
    #     if self.q0:
    #         self.sys.q = {
    #             'xm' : q0[0],
    #             'ym' : q0[1],
    #             'xr' : q0[2],
    #             'r' : q0[3],
    #             }
    #     self.sys.satisfy_constraints()
    #     ## del self.mvi
    #     ## self.mvi = trep.MidpointVI(self.sys)
    #     ## self.mvi.initialize_from_configs(0,self.q0,DT,self.q0)
    #     self.dsys.set(self.dsys.build_state(Q=self.sys.q),
    #                   self.dsys.build_input(rho=self.q0[2:]),
    #                   0)
    #     return




# class Filter:
#     """
#     This class is the base class for my EKF.  It must contain methods
#     for instantiating a filter, modifying covariances, updating the
#     filter, and returning any of the private filter parameters.
#     """
#     def __init__(self):

#         return

#     def set_meas_cov(self, cov):

#         return

#     def set_model_cov(self, cov):

#         return



class System:
    """
    This class will be responsible for creating an instance of the
    system, an instance of the filter, and reading in the results of
    an optimization to obtain controller gains and reference
    trajectories. It will also define all publishers and subscribers,
    and contain any callbacks.
    """
    def __init__(self):
        rospy.loginfo("Starting closed-loop controller...")
        # first we must parse the CL args:
        args = sys.argv
        # find reference trajectory
        abbreviations = {
            '-f' : '--filename'
            }
        ## replace abbreviations with full name
        for i in range(len(args)):
            if args[i] in abbreviations:
                args[i] = abbreviations[args[i]]
        if '--filename' not in args:
            # then we just exit
            rospy.logerr("Must provide a filename!")
            sys.exit(1)
        else:
            fname = args[args.index('--filename')+1]
            if not os.path.isfile(fname):
                rospy.logerr("Filename not found: ",fname)
                sys.exit(1)

        # now we can load in the traj:
        self.system = MassSystem2D()
        (self.tref, self.Qref, self.pref, self.vref, self.uref, self.rhoref) = \
            trep.load_trajectory(fname, self.system.sys)
        # now we can create the dsys object:
        self.system.create_dsys(self.tref)
        # combine reference trajectory into state and input references:
        (self.Xref, self.Uref) = self.system.dsys.build_trajectory(
            Q=self.Qref, p=self.pref, v=self.vref, u=self.uref, rho=self.rhoref)
        # now we can initialize the variational integrator using the
        # reference trajectory as our guide
        self.system.dsys.set(self.Xref[0], self.Uref[0], 0)
        rospy.loginfo("trep discopt system created and " \
                      "variational integrator initialized")

        # Now we can linearize the trajectory, and find control gains:
        # first we need the cost functions (let's start with identity)
        def Qfunc(kf): return np.diag([1, 1, 1, 1, 1, 1, 1, 1])
        def Rfunc(kf): return np.diag([1, 1])
        (self.Kproj, self.Avec, self.Bvec) = self.system.dsys.calc_feedback_controller(
            self.Xref, self.Uref, Q=Qfunc, R=Rfunc, return_linearization=True)
        rospy.loginfo("Successfully found linearization and feedback law")

        # now we can define our filter parameters:
        self.meas_cov = np.diag((0.5,0.5,0.5,0.5,0.75,0.75,0.75,0.75)) # measurement covariance
        self.proc_cov = np.diag((0.1,0.1,0.1,0.1,0.15,0.15,0.15,0.15)) # process covariance
        self.est_cov = self.meas_cov # estimate covariance

        # now we can define all callbacks, publishers, and subscribers:
        self.meas_sub = rospy.Subscriber("meas_config", PlanarSystemConfig, self.meascb)
        self.filt_pub = rospy.Publisher("filt_config", PlanarSystemConfig)
        self.comm_pub = rospy.Publisher("serial_commands", RobotCommands)
        
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
