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
    - PlanarSystemConfig (ref_config)
    - RobotCommands     (serial_commands)
    - Path (mass_ref_path)
    - PlanarCovariance (post_covariance)

SERVICES:
    - PlanarSystemService (get_ref_config)

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
from puppeteer_msgs.srv import PlanarSystemService
from puppeteer_msgs.msg import PlanarCovariance
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import trep
from trep import tx, ty, tz, rx, ry, rz
import trep.discopt as discopt
from math import sin, cos
from math import pi as mpi
import numpy as np
import sys
import scipy as sp
import os
import copy
import time
from scipy.interpolate import interp1d as spline

## define some global constants:
BALL_MASS = 0.1244 ## kg
g = 9.81 ## m/s^2
h0 = 1 ## default height of robot in m
DT = 1/30.0 ## nominal dt for the system
CALLBACK_DIVISOR = 30

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
                rospy.logerr("Invalid number of initial conditions")
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
            print "filename is ",fname
            if not os.path.isfile(fname):
                rospy.logerr("Filename not found: %s",fname)
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
        def Qfunc(kf): return np.diag([10, 10, 1, 1, 1, 1, 1, 1])
        def Rfunc(kf): return np.diag([1, 1])
        (self.Kproj, self.Avec, self.Bvec) = self.system.dsys.calc_feedback_controller(
            self.Xref, self.Uref, Q=Qfunc, R=Rfunc, return_linearization=True)
        rospy.loginfo("Successfully found linearization and feedback law")

        # now we can define our filter parameters:
        self.meas_cov = np.diag((0.5,0.5,0.5,0.5,0.75,0.75,0.75,0.75)) # measurement covariance
        self.proc_cov = np.diag((0.1,0.1,0.1,0.1,0.15,0.15,0.15,0.15)) # process covariance
        self.est_cov = copy.deepcopy(self.meas_cov) # estimate covariance

        # now we can define all callbacks, publishers, subscribers,
        # services, and parameters:
        self.meas_sub = rospy.Subscriber("meas_config", PlanarSystemConfig,
                                         self.meascb)
        self.filt_pub = rospy.Publisher("filt_config", PlanarSystemConfig)
        self.ref_pub = rospy.Publisher("ref_config", PlanarSystemConfig)
        self.comm_pub = rospy.Publisher("serial_commands", RobotCommands)
        self.path_pub = rospy.Publisher("mass_ref_path", Path)
        self.ref_pub = rospy.Publisher("ref_config", PlanarSystemConfig)
        self.cov_pub = rospy.Publisher("post_covariance", PlanarCovariance)
        self.conf_serv = rospy.Service("get_ref_config", PlanarSystemService,
                                       self.ref_config_service_handler)
        if rospy.has_param("robot_index"):
            self.robot_index = rospy.get_param("robot_index")
        else:
            self.robot_index = 0
            rospy.set_param("robot_index", self.robot_index)
        rospy.set_param("/operating_condition", 0)


        # send the robot it's starting pose in the /optimization_frame
        rospy.logwarn("Waiting for three seconds!!!")
        time.sleep(3)
        rospy.loginfo("Ready to go!!!")
        self.send_initial_config()
        # send robot a start command:
        self.send_start_command()

        # now let's initialize a bunch of variables that many of the
        # other methods in this class will need access to
        self.tbase = rospy.Time.from_sec(0)
        self.k = int(0)
        self.t1 = self.t2 = 0
        self.u1 = np.zeros((self.system.sys.nQk, 1))
        self.u2 = np.zeros((self.system.sys.nQk, 1))
        self.Qmeas1 = np.zeros((self.system.sys.nQ, 1))
        self.Qmeas2 = np.zeros((self.system.sys.nQ, 1))
        self.Xmeas1 = np.zeros((self.system.dsys.nX, 1))
        self.Xmeas2 = np.zeros((self.system.dsys.nX, 1))
        self.Xpred1 = np.zeros((self.system.dsys.nX, 1))
        self.Xpred2 = np.zeros((self.system.dsys.nX, 1))
        self.Xest1 = np.zeros((self.system.dsys.nX, 1))
        self.Xest2 = np.zeros((self.system.dsys.nX, 1))
        self.first_flag = True
        self.callback_count = 0

        self.send_reference_path()

        ## let's check for the existence of a frequency param.
        ## Otherwise, let's use the default
        if rospy.has_param("controller_freq"):
            tmp = rospy.get_param("controller_freq")
            self.dt = 1/float(tmp)
        else:
            tmp = 1/DT
            self.dt = DT
            rospy.set_param("controller_freq", tmp)
        rospy.loginfo("Controller frequency = %f Hz (dt = %f sec.)", tmp, self.dt)

        # check that read in mat file is approximately the same
        # frequency as the desired:
        if np.abs(self.dt - (self.tref[1]-self.tref[0])) >= 0.001:
            rospy.logerr("parameter frequency and mat file frequency don't match!")
            rospy.logerr("parameter = %f s   mat file = %f s",self.dt,
                         (self.tref[1]-self.tref[0]))
            rospy.signal_shutdown("Frequency Mismatch!")
        # idle on startup:
        rospy.set_param("/operating_condition", 0)

        return



    def send_initial_config(self):
        com = RobotCommands()
        com.robot_index = self.robot_index
        com.type = ord('a')
        com.header.stamp = rospy.get_rostime()
        com.header.frame_id = "/optimization_frame"
        com.div = 4
        com.x = self.Qref[0][2]
        com.y = 0
        com.th = 0
        com.height_left = self.Qref[0][3]
        com.height_right = 1
        self.comm_pub.publish(com)
        return



    def ref_config_service_handler(self, req):
        """
        Return a reference configuration for the system by either
        index or time
        """
        if req.t != 0:
            # then use time, otherwise use the index
            try:
                index = [i for i,x in enumerate(self.tref) \
                         if x <= req.t < self.tref[i+1]][0]
            except:
                rospy.logerr("Invalid time for service request!")
                return None
            if index > len(self.tref)-1:
                rospy.logerr("Requested index is too large!")
                return None
            config = PlanarSystemConfig()
            config.xm = self.Qref[index][0]
            config.ym = self.Qref[index][1]
            config.xr = self.Qref[index][2]
            config.r  = self.Qref[index][3]
            return {'config': config}
        else:
            # use the index:
            if req.index > len(self.tref)-1:
                rospy.logerr("Requested index is too large!")
                return None
            else:
                config = PlanarSystemConfig()
                config.xm = self.Qref[req.index][0]
                config.ym = self.Qref[req.index][1]
                config.xr = self.Qref[req.index][2]
                config.r  = self.Qref[req.index][3]
                return {'config': config}
        return None



    def send_reference_path(self):
        path = Path()
        path.header.frame_id = "/optimization_frame"
        for i in range(len(self.tref)):
            pose = PoseStamped()
            pose.header.frame_id = path.header.frame_id
            pose.pose.position.x = self.Qref[i][0]
            pose.pose.position.y = self.Qref[i][1]
            pose.pose.position.z = 0
            path.poses.append(pose)
        self.path_pub.publish(path)
        return




    def copy_two_to_one(self):
        """
        This function merely performs a deep copy for all of the "2"
        variables into the "1" variables.
        """
        self.t1 = copy.deepcopy(self.t2)
        self.Qmeas1 = copy.deepcopy(self.Qmeas2)
        self.Xmeas1 = copy.deepcopy(self.Xmeas2)
        self.Xpred1 = copy.deepcopy(self.Xpred2)
        self.Xest1 = copy.deepcopy(self.Xest2)
        return



    def calc_send_controls(self):
        self.u1 = self.Xest2[2:4]
        self.u2 = self.Uref[self.k] + \
          matmult(self.Kproj[self.k], self.Xref[self.k]-self.Xest2)
        # now convert to a velocity and send out:
        ucom = (self.u2-self.u1)/self.dt
        com = RobotCommands()
        com.robot_index = self.robot_index
        com.type = ord('i')
        com.v_robot = ucom[0]
        com.w_robot = 0
        com.rdot = 0
        com.rdot_left = ucom[1]
        com.rdot_right = 0
        com.div = 3
        self.comm_pub.publish(com)
        return


    def publish_covariance(self):
        """
        This function simply publishes the updated covariance from the
        EKF.
        """
        tmp_cov = copy.deepcopy(self.est_cov)
        tmp_cov = np.ravel(tmp_cov)
        cov = PlanarCovariance()
        for i,val in enumerate(tmp_cov):
            cov.covariance[i] = val
        self.cov_pub.publish(cov)
        del tmp_cov, cov
        return



    def stop_robots(self):
        com = RobotCommands()
        com.robot_index = self.robot_index
        com.type = ord('q')
        com.header.stamp = rospy.get_rostime()
        self.comm_pub.publish(com)
        return


    def send_start_command(self):
        com = RobotCommands()
        com.robot_index = self.robot_index
        com.type = ord('m')
        com.header.stamp = rospy.get_rostime()
        self.comm_pub.publish(com)
        return


    def get_gen_mom(self):
        """ Initialize a VI, and calculate the discrete Legendre
        transform, then return the momenta array """
        self.system.mvi.q1 = self.Qmeas1
        self.system.mvi.q2 = self.Qmeas2
        self.system.mvi.t1 = self.t1
        self.system.mvi.t2 = self.t2
        self.system.mvi.calc_p2()
        p2 = self.system.mvi.p2
        return p2


    def get_kin_vel(self):
        """ Find the finite difference velocities of the kinematic
        configuration variables, and return the array """
        qk = self.Qmeas2[2:]
        qkm = self.Qmeas1[2:]
        return ((qk-qkm)/(self.t2-self.t1))


    def get_prediction(self):
        """ Use the variational integrator to
        step forward in time, and predict where we should be, and
        return that array """
        self.system.mvi.initialize_from_state(self.t1,
                                              self.Xest1[0:4],
                                              self.Xest1[4:6])
        # let's interpolate/extrapolate the u2 to get the actual one
        # i.e. let's account for the fact that dt is not actually
        # constant:
        con = self.u1 + ((self.u2-self.u1)/self.dt)*(self.t2-self.t1)
        self.system.mvi.step(self.t2, u1=(), k2=con)
        v2 = (self.system.mvi.q2[2:]-self.system.mvi.q1[2:])/ \
          (self.system.mvi.t2-self.system.mvi.t1)
        tmp = np.hstack((self.system.mvi.q2,
                         self.system.mvi.p2,
                         v2))
        return tmp



    def locally_linearize(self):
        """
        This function will locally linearize a dsys object around the
        current prediction.  Note that the variational integrator
        contained in self.system.mvi is already set correctly to do
        the correct linearization.  So instead of calling the
        linearize_trajectory method, I will build the matrices myself.
        """
        # The mvi was initialized at X1 in self.get_prediction(), and
        # it was stepped to t2, using the correct input.  So all we
        # need to do is set the dsys times and index
        self.system.dsys.time = np.array([self.t1, self.t2])
        self.system.dsys._k = 0 # BAD PRACTICE!!!
        return self.system.dsys.fdx()



    def update_filter(self):
        """ Run the EKF equations to update the kalman filter, and
        return the posterior mean and covariance """
        # predict estimated covariance:
        xkm1 = self.Xpred2
        Atmp = self.locally_linearize()
        Fkm1 = Atmp.copy()
        # Fkm1 = self.Avec[self.k]
        Pkm1 = matmult(Fkm1, self.est_cov, Fkm1.T) + self.proc_cov
        # innovation:
        yk = self.Xmeas2 - xkm1
        # residual covariance:
        Sk = Pkm1 + self.meas_cov
        # Kalman gain:
        Kk = matmult(Pkm1, np.linalg.inv(Sk))
        # updated state estimate:
        xkk = xkm1 + matmult(Kk, yk)
        tmp = np.identity(self.system.dsys.nX)-Kk
        Pkk = matmult(tmp, Pkm1)
        # use following lines for disabling the filter:
        # xkk = self.Xmeas2
        # Pkk = self.est_cov
        return (xkk, Pkk)



    # function for publishing reference and filtered information:
    def send_filt_and_ref(self, data):
        # publish the estimated pose:
        qest = PlanarSystemConfig()
        qest.header.stamp = data.header.stamp
        qest.header.frame_id = data.header.frame_id
        qest.xm = self.Xest2[0]
        qest.ym = self.Xest2[1]
        qest.xr = self.Xest2[2]
        qest.r  = self.Xest2[3]
        self.filt_pub.publish(qest)
        # publish the reference pose:
        qest = PlanarSystemConfig()
        qest.header.stamp = data.header.stamp
        qest.header.frame_id = data.header.frame_id
        qest.xm = self.Qref[self.k][0]
        qest.ym = self.Qref[self.k][1]
        qest.xr = self.Qref[self.k][2]
        qest.r  = self.Qref[self.k][3]
        self.ref_pub.publish(qest)
        return



    # define the callback function for the estimate out of the kinect
    def meascb(self, data):
        rospy.logdebug("measurement callback triggered")

        # first, let's get the operating condition and act
        # appropriately:
        op = rospy.get_param("/operating_condition")
        # increment callback count
        self.callback_count += 1;

        if (op != 2):
            # we are not running, so let's just keep running the
            # initializations and exiting
            self.first_flag = True
            # once in a while, let's send the reference path
            if not (self.callback_count%CALLBACK_DIVISOR):
                self.send_reference_path()
            return

        if self.first_flag:
            rospy.loginfo("Beginning trajectory")
            self.send_initial_config()
            self.send_reference_path()
            # this is the first run through the callback while in the
            # running state, so let's initialize all of the variables,
            # and then make sure to set the flag back to false at the
            # end of the callback
            self.tbase = data.header.stamp
            self.t2 = 0.0
            self.k = 0
            # fill out X2 stuff:
            self.Qmeas2 = np.array([data.xm, data.ym, data.xr, data.r])
            # let's assume that both the discrete generalized momenta
            # and the kinematic configuration variables are zero
            # because we are at rest at this point
            self.Xmeas2 = np.hstack((self.Qmeas2, np.zeros(self.system.sys.nQ)))
            # our initial prediction for the state is just the initial
            # state of the system:
            self.Xpred2 = self.Xref[self.k]
            # # for our initial estimate, let's just average the
            # # prediction and the measurement
            # self.Xest2 = (self.Xmeas2+self.Xpred2)/2.0
            self.Xest2 = self.Xmeas2
            # now run our control law
            self.calc_send_controls()
            # send filter and reference info:
            self.send_filt_and_ref(data)
            # send out nominal (feedforward) controls:
            self.first_flag = False
            return

        # let's increment our index:
        self.k += 1
        # do we need to exit?
        if self.k >= len(self.tref)-1:
            rospy.loginfo("Trajectory finished...")
            self.stop_robots()
            self.first_flag = True
            rospy.set_param("/operating_condition", 3)
            return
        # copy X2 stuff to X1:
        self.copy_two_to_one()
        # calculate and fill out new X2 stuff:
        self.t2 = (data.header.stamp-self.tbase).to_sec()
        self.Qmeas2 = np.array([data.xm, data.ym, data.xr, data.r])
        ptmp = self.get_gen_mom()
        vtmp = self.get_kin_vel()
        self.Xmeas2 = np.hstack((self.Qmeas2, ptmp, vtmp))
        # now we need to make our prediction:
        self.Xpred2 = self.get_prediction()
        # Update the EKF, and get the posterior mean:
        (self.Xest2, self.est_cov) = self.update_filter()
        # run control law and send controls:
        self.calc_send_controls()
        # publish the covariance:
        self.publish_covariance()
        # send filter and reference info:
        self.send_filt_and_ref(data)

        return


def main():
    """
    Run the main loop, by instatiating a System class, and then
    calling ros.spin
    """
    rospy.init_node('cl_control', log_level=rospy.INFO)

    try:
        sim = System()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
