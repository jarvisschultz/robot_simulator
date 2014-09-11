#!/usr/bin/env python
"""
Jarvis Schultz
September 2013

This program implements a full closed-loop receding horizon controller plus EKF
estimator for the planar mass system.  Every time a new measurement is received,
the estimator produces a new estimate for the full state of the system.  A
service is then provided to get the next reference state.  Then a full nonlinear
trajectory optimization is performed to determine the proper controls to send to
the robot.  

SUBSCRIPTIONS:
    - PlanarSystemConfig (meas_config)

PUBLISHERS:
    - PlanarSystemConfig (filt_config)
    - PlanarSystemConfig (ref_config)
    - RobotCommands (serial_commands)
    - Path (mass_ref_path)
    - Path (mass_filt_path)
    - PlanarCovariance (post_covariance)
    - PlanarSystemState (filt_state)
    - Time (start_time)

SERVICES:
    - PlanarSystemService (get_ref_config) (client)
    - PlanarStateAbsTime (get_ref_state) (client)

NOTES:
    - Let's assume that the configurations that are received in this node are
    guaranteed to be in the right coordinate system.

"""

## define all imports:
import roslib; roslib.load_manifest('robot_simulator')
import rospy
import tf
from std_msgs.msg import Time
from puppeteer_msgs.msg import FullRobotState
from puppeteer_msgs.msg import PlanarSystemConfig
from puppeteer_msgs.msg import RobotCommands
from puppeteer_msgs.msg import PlanarCovariance
from puppeteer_msgs.msg import PlanarSystemState
from puppeteer_msgs.srv import PlanarSystemService
from puppeteer_msgs.srv import PlanarSystemServiceRequest
from puppeteer_msgs.srv import PlanarStateAbsTime
from puppeteer_msgs.srv import PlanarStateAbsTimeRequest
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
from collections import deque
import scipy.io as sio

## define some global constants:
BALL_MASS = 0.1244 ## kg
g = 9.81 ## m/s^2
h0 = 1 ## default height of robot in m
DT = 1/30.0 ## nominal dt for the system
CALLBACK_DIVISOR = 3
PATH_TIME = 6.0
PATH_LENGTH = int(PATH_TIME*1/DT)
WINDOW = 16

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
                
        # create a system
        self.system = MassSystem2D()

        # now we need to wait for the ref_config service to be available.  That
        # way we can initialize the filter
        rospy.loginfo("Waiting for reference config service to be available...")
        rospy.wait_for_service("get_ref_config")
        self.get_ref_config = rospy.ServiceProxy("get_ref_config", PlanarSystemService)
        rospy.loginfo("reference config service now available.")
        req = PlanarSystemServiceRequest(index=0)
        resp = self.get_ref_config(req)
        # set initial state and config:
        self.Xref = np.array([resp.state]*WINDOW)
        self.Xref_init = resp.state
        self.Qref = resp.state[0:self.system.sys.nQ]
        self.Qref_init = resp.state[0:self.system.sys.nQ]
        self.Uref = np.array([resp.input]*(WINDOW-1))
        # self.Config = resp.config

        # wait for service for the reference state
        rospy.loginfo("Waiting for reference state service to be available...")
        rospy.wait_for_service("get_ref_state")
        self.get_ref_state = rospy.ServiceProxy("get_ref_state", PlanarStateAbsTime)
        rospy.loginfo("reference state service now available.")

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
        # self.tref = np.array([0.0, self.dt])
        self.tref = np.arange(0, WINDOW*self.dt + self.dt, self.dt)
        # now we can create the dsys object:
        self.system.create_dsys(self.tref)
        self.system.dsys.set(np.array(self.Xref[0]), np.array(self.Uref[0]), 0)
        rospy.loginfo("trep discopt system created and " \
                      "variational integrator initialized")

        # check that read in mat file is approximately the same
        # frequency as the desired:
        if np.abs(self.dt - resp.dt) >= 0.001:
            rospy.logerr("parameter frequency and mat file frequency don't match!")
            rospy.logerr("parameter = %f s   mat file = %f s",self.dt,
                         (self.tref[1]-self.tref[0]))
            rospy.signal_shutdown("Frequency Mismatch!")
        # idle on startup:
        rospy.set_param("/operating_condition", 0)

        # now we can define our filter parameters:
        self.meas_cov = np.diag((0.5,0.5,0.5,0.5,1000,1000,1000,1000)) # measurement covariance
        self.proc_cov = np.diag((0.1,0.1,0.1,0.1,0.15,0.15,0.15,0.15)) # process covariance
        self.est_cov = copy.deepcopy(self.meas_cov) # estimate covariance

        # define cost function parameters
        # self.Qcost = np.diag([1, 1, 0.1, 0.1, 0.1, 0.1, 50000, 50000])
        self.Qcost = np.diag([10, 10, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        self.Rcost = np.diag([0.1, 0.1])
            
        # now we can define all callbacks, publishers, subscribers,
        # services, and parameters:
        self.meas_sub = rospy.Subscriber("meas_config", PlanarSystemConfig,
                                         self.meascb)
        self.time_pub = rospy.Publisher("start_time", Time)
        self.filt_pub = rospy.Publisher("filt_config", PlanarSystemConfig)
        self.filt_state_pub = rospy.Publisher("filt_state", PlanarSystemState)
        self.ref_pub = rospy.Publisher("ref_config", PlanarSystemConfig)
        self.comm_pub = rospy.Publisher("serial_commands", RobotCommands)
        self.ref_path_pub = rospy.Publisher("mass_ref_path", Path)
        self.filt_path_pub = rospy.Publisher("mass_filt_path", Path)
        self.cov_pub = rospy.Publisher("post_covariance", PlanarCovariance)
        if rospy.has_param("robot_index"):
            self.robot_index = rospy.get_param("robot_index")
        else:
            self.robot_index = 0
            rospy.set_param("robot_index", self.robot_index)

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
        self.full_ref_flag = False
        self.ref_lag_count = 0
        self.callback_count = 0
        self.Xfilt_vec = deque([], maxlen=PATH_LENGTH)
        self.Xref_vec = deque([], maxlen=PATH_LENGTH)
        
        return



    def send_initial_config(self):
        com = RobotCommands()
        com.robot_index = self.robot_index
        com.type = ord('a')
        com.header.stamp = rospy.get_rostime()
        com.header.frame_id = "/optimization_frame"
        com.div = 4
        com.x = self.Qref_init[2]
        com.y = 0
        com.th = 0
        com.height_left = self.Qref_init[3]
        com.height_right = 1
        self.comm_pub.publish(com)
        return


    def send_filt_path(self, data):
        path = Path()
        path.header.stamp = data.header.stamp
        path.header.frame_id = data.header.frame_id
        pose = PoseStamped()
        pose.header.frame_id = path.header.frame_id
        pose.pose.position.x = self.Xest2[0]
        pose.pose.position.y = self.Xest2[1]
        pose.pose.position.z = 0
        self.Xfilt_vec.append(pose)
        path.poses = list(self.Xfilt_vec)
        self.filt_path_pub.publish(path)
        return


    def send_reference_path(self, data):
        path = Path()
        path.header.stamp = data.header.stamp
        path.header.frame_id = data.header.frame_id
        pose = PoseStamped()
        pose.header.frame_id = path.header.frame_id
        pose.pose.position.x = self.Qref[0, 0]
        pose.pose.position.y = self.Qref[0, 1]
        pose.pose.position.z = 0
        self.Xref_vec.append(pose)
        path.poses = list(self.Xref_vec)
        self.ref_path_pub.publish(path)
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


    def state_to_array(self, state):
        """
        convert a PlanarSystemState message to an array of appropriate length
        and in the correct order
        """
        out = np.zeros(self.system.dsys.nX)
        n = self.system.sys.nQ
        out[self.system.sys.get_config('xm').index] = state.xm
        out[self.system.sys.get_config('ym').index] = state.ym
        out[self.system.sys.get_config('xr').index] = state.xr
        out[self.system.sys.get_config('r').index] = state.r
        out[self.system.sys.get_config('xm').index + n] = state.pxm
        out[self.system.sys.get_config('ym').index + n] = state.pym
        out[self.system.sys.get_config('xr').index + n] = state.vxr
        out[self.system.sys.get_config('r').index + n] = state.vr
        return out
        

    def calc_send_controls(self, tnext):
        self.u1 = self.Xest2[2:4]
        # first request the reference state at the next time step:
        try:
            req = PlanarStateAbsTimeRequest(tnext)
            resp = self.get_ref_state(req)
            # do we need to exit?
            if resp.stop:
                rospy.loginfo("Trajectory finished...")
                self.stop_robots()
                self.first_flag = True
                rospy.set_param("/operating_condition", 3)
                return
            self.Xref = np.vstack((self.Xref, self.state_to_array(resp.state)))[1:]
            self.Qref = self.Xref[:,0:self.system.sys.nQ]
        except rospy.ServiceException, e:
            print "Service did not process request: %s"%str(e)
            rospy.logerr("No reference provided to control node")
            self.stop_robots()
            rospy.set_param("/operating_condition", 4)
            return
        # now we can perform the full trajectory optimization to obtain the
        # correct inputs to send to the system
        # first get an initial guess:
        self.system.dsys.time = self.tref
        Xd = np.vstack((self.Xest2, self.Xref))
        Ud = self.Xref[:,2:4]
        self.system.dsys.set(self.Xest2, self.Xref[0,2:4], 0)
        X0 = np.array([self.Xest2])
        U0 = np.array([self.Xref[0,2:4]])
        for i,u in enumerate(Ud[1:]):
            X0 = np.vstack((X0, self.system.dsys.f()))
            U0 = np.vstack((U0, u))
            self.system.dsys.step(u)
        X0 = np.vstack((X0, self.system.dsys.f()))
        X0_init = X0.copy()
        U0_init = U0.copy()
        Xd[:,6:8] = 0
        cost = discopt.DCost(Xd, Ud, self.Qcost, self.Rcost)
        optimizer = discopt.DOptimizer(self.system.dsys, cost)
        optimizer.optimize_ic = False
        optimizer.descent_tolerance = 1e-1
        optimizer.first_method_iterations = 0
        optimizer.armijo_beta = 0.99
        finished = False
        optimizer.monitor = discopt.DOptimizerMonitor()
        # now we can actually perform the trajectory optimization
        try:
            step_count = 0
            while not finished:
                if step_count == 0: method='steepest'
                else: method='steepest'
                finished,X0,U0,dcost,ncost = optimizer.step(
                    step_count, X0, U0, method=method)
                step_count += 1
                if rospy.Time.now() > tnext:
                    rospy.logwarn("optimization time elapsed = "\
                                   "{0:4.4f} s for {1:d} steps".format(
                                       (rospy.Time.now()-tnext).to_sec(), step_count, i))
                    break
        except trep.ConvergenceError as e:
            rospy.logwarn("Detected optimization problem: %s"%e.message)
            rospy.logerr("Failed to optimize... exiting...")
            self.stop_robots()
            rospy.set_param("/operating_condition", 4)
            rospy.signal_shutdown("Optimization failure")
        except:
            rospy.logerr("Unknown error... exiting...")
            self.stop_robots()
            rospy.set_param("/operating_condition", 4)
            rospy.signal_shutdown("Optimization failure")

        # if (rospy.Time.now() - self.tbase).to_sec() > 1.5:
        #     fname = '/home/jarvis/Desktop/misc/debug_data/receding_debug/true_dat.mat'
        #     dat = {}
        #     dat['X'] = X0
        #     dat['Xd'] = Xd
        #     dat['X0'] = X0_init
        #     dat['U'] = U0
        #     dat['Ud'] = Ud
        #     dat['U0'] = U0_init
        #     dat['tref'] = self.tref
        #     sio.savemat(fname, dat)
        #     rospy.signal_shutdown("done")

        # now convert the optimal trajectory into a set of controls for the
        # robot
        self.u2 = U0[0]
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
        # publish estimated state
        xest = PlanarSystemState()
        xest.header.stamp = data.header.stamp
        xest.header.frame_id = data.header.frame_id
        xest.xm = self.Xest2[0]
        xest.ym = self.Xest2[1]
        xest.xr = self.Xest2[2]
        xest.r  = self.Xest2[3]
        xest.pxm = self.Xest2[4]
        xest.pym = self.Xest2[5]
        xest.vxr = self.Xest2[6]
        xest.vr  = self.Xest2[7]
        self.filt_state_pub.publish(xest)
        # publish the reference pose:
        qest = PlanarSystemConfig()
        qest.header.stamp = data.header.stamp
        qest.header.frame_id = data.header.frame_id
        qest.xm = self.Qref[0]
        qest.ym = self.Qref[1]
        qest.xr = self.Qref[2]
        qest.r  = self.Qref[3]
        self.ref_pub.publish(qest)
        # publish the path info
        self.send_reference_path(data)
        self.send_filt_path(data)
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
            self.full_ref_flag = False
            self.ref_lag_count = 0
            if len(self.Xfilt_vec):
                self.Xfilt_vec.clear()
                self.Xref_vec.clear()
            return

        if self.first_flag:
            rospy.loginfo("Beginning trajectory")
            # send starting config
            self.send_initial_config()
            # this is the first run through the callback while in the
            # running state, so let's initialize all of the variables,
            # and then make sure to set the flag back to false at the
            # end of the callback
            self.tbase = data.header.stamp
            # publish base time:
            self.time_pub.publish(self.tbase)
            self.t2 = 0.0
            self.first_flag = False
            return
        if not self.full_ref_flag:
            # try and fill up the array
            while self.ref_lag_count < WINDOW:
                try:
                    req = PlanarStateAbsTimeRequest(self.tbase +
                        rospy.Duration.from_sec(self.ref_lag_count+1)*self.dt)
                    resp = self.get_ref_state(req)
                    if not resp.stop:
                        self.Xref[self.ref_lag_count] = self.state_to_array(
                            resp.state)
                        self.Qref = self.Xref[:,0:self.system.sys.nQ]
                        self.ref_lag_count += 1
                    else:
                        return
                except rospy.ServiceException, e:
                    rospy.logwarn("Service did not process request: %s"%str(e))
                    return
            self.full_ref_flag = True
            # fill out X2 stuff:
            self.Qmeas2 = np.array([data.xm, data.ym, data.xr, data.r])
            # let's assume that both the discrete generalized momenta
            # and the kinematic configuration variables are zero
            # because we are at rest at this point
            self.Xmeas2 = np.hstack((self.Qmeas2, np.zeros(self.system.sys.nQ)))
            # our initial prediction for the state is just the initial
            # state of the system:
            self.Xpred2 = self.Xref[0]
            # # for our initial estimate, let's just average the
            # # prediction and the measurement
            # self.Xest2 = (self.Xmeas2+self.Xpred2)/2.0
            self.Xest2 = self.Xmeas2
            # now run our control law
            self.calc_send_controls(data.header.stamp +
                                    rospy.Duration.from_sec(self.dt*WINDOW))
            # send filter and reference info:
            self.send_filt_and_ref(data)
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
        self.calc_send_controls(data.header.stamp + 
                                rospy.Duration.from_sec(self.dt*WINDOW))
        # publish the covariance:
        self.publish_covariance()
        # send filter and reference info:
        self.send_filt_and_ref(data)
        # once in a while, let's send the reference path
        if not (self.callback_count%CALLBACK_DIVISOR):
            self.send_reference_path(data)
            self.send_filt_path(data)

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
