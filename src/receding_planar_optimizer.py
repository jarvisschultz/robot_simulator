#!/usr/bin/env python
"""
Jarvis Schultz
November 2012

This program subscribes to the output of an EKF published by
receding_planar_controller.py.  It builds its own local copies of the complete
system trajectory.  Then as the system is running, it repeatedly re-solves the
optimization problem and provides access to its version of the reference
trajectory and its controller via services.

SUBSCRIPTIONS:
    - PlanarSystemState (filt_state)

PUBLISHERS:
    - PlanarControlLaw (receding_controls)

SERVICES:
    - PlanarSystemService (get_receding_config) (client)
    - PlanarControlLawService (get_receding_controller) (provider)

"""

## ros imports:
import roslib; roslib.load_manifest('robot_simulator')
import rospy
import tf
from puppeteer_msgs.msg import FullRobotState
from puppeteer_msgs.msg import PlanarSystemConfig
from puppeteer_msgs.msg import PlanarSystemState
from puppeteer_msgs.msg import RobotCommands
from puppeteer_msgs.srv import PlanarSystemService
from puppeteer_msgs.srv import PlanarSystemServiceRequest
from puppeteer_msgs.srv import PlanarControlLawService
from puppeteer_msgs.msg import PlanarControlLaw
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import MultiArrayDimension

## trep and misc.
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
import threading

## internal
import receding_planar_controller as rp


## PRIMARY CLASS WITH CALLBACKS ##############################
class RecedingOptimizer:
    """
    This class will define all subscribers and service handlers.
    """
    def __init__(self):
        rospy.loginfo("Starting receding-horizon optimizer...")
        # let's create a local instance of the MassSystem2D() that we can
        # perform our optimizations with:
        self.system = rp.MassSystem2D()

        # let's wait for and call the get_ref_config service to get the full
        # reference trajectory information from receding_planar_controller.py:
        rospy.loginfo("Waiting for reference service to be available...")
        rospy.wait_for_service("get_ref_config")
        get_ref_config = rospy.ServiceProxy("get_ref_config", PlanarSystemService)

        req = PlanarSystemServiceRequest(index=0)
        resp = get_ref_config(req)
        self.Xref = [resp.state]
        self.Uref = [resp.input]
        self.dt = resp.dt
        self.tref = [resp.time]
        self.length = resp.length
        # now we can fully fill in the reference trajectory
        for i in range(1, self.length-1):
            req = PlanarSystemServiceRequest(index=i)
            resp = get_ref_config(req)
            self.Xref.append(resp.state)
            self.Uref.append(resp.input)
            self.tref.append(resp.time)
        req = PlanarSystemServiceRequest(index=self.length-1)
        resp = get_ref_config(req)
        self.Xref.append(resp.state)
        self.tref.append(resp.time)
        rospy.loginfo("Optimizer successfully received entire trajectory")

        # define all publishers:
        self.cont_pub = rospy.Publisher("receding_controls", PlanarControlLaw)
        # define all service providers:
        self.cont_serv = rospy.Service("get_receding_controller",
                                       PlanarControlLawService,
                                       self.control_service_handler)
        # define all subscribers:
        self.filt_sub = rospy.Subscriber("filt_state", PlanarSystemState,
                                         self.meascb)
        # define miscellaneous variables related to the receding horizon controller:
        self.system.create_dsys(np.array(self.tref))
        # perform linearization about current trajectory so that I can give it
        # to any service requesters:
        self.K, self.A, self.B = self.system.dsys.calc_feedback_controller(
            np.array(self.Xref), np.array(self.Uref), return_linearization=True)

        # initialize all variables that are needed for the receding horizon
        # optimization
        self.Xfilt = []
        self.first_flag = True
        self.base_time = rospy.Time.from_sec(0)
        self.Xfilt_len = 0
        # manually open up another thread that repeatedly performs the full
        # discrete optimization as fast as possible:
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.optimizer)
        self.thread.start()
        return



    def optimizer(self):
        """
        This is a function that is given its own thread, and it will continually
        build a new reference trajectory and re-optimize.
        """
        print("started the optimizer function in %s"%
                       threading.currentThread().getName())
        while not rospy.is_shutdown():
            # print("started the optimizer function in %s"%
            #                threading.currentThread().getName())
            # print("lock status = %s"%self.lock.locked())
            # first let's check if the length of the filtered vector is longer than
            # the last time we did the optimization
            with self.lock:
                Xfilt_local = copy.deepcopy(self.Xfilt)
                Xfilt_len_local = copy.deepcopy(self.Xfilt_len)
                Xref_local = copy.deepcopy(self.Xref)
                Uref_local = copy.deepcopy(self.Uref)
            if len(Xfilt_local) > Xfilt_len_local:
                with self.lock:
                    self.Xfilt_len = len(Xfilt_local)
            else:
                rospy.logdebug("No new information for the optimization...")
                rospy.sleep(1/30.)
                continue

            # if we got here, we can do a new optimization:
            Qcost = np.diag([10000, 10000, 1, 1, 1000, 1000, 300, 300])
            Rcost = np.diag([1, 1])
            # build trajectories:
            Xref = np.vstack((Xfilt_local, Xref_local[len(Xfilt_local):]))
            Uref = Xref[1:,2:4]
            cost = discopt.DCost(np.array(Xref), np.array(Uref), Qcost, Rcost)
            # print("lock status = %s"%self.lock.locked())
            with self.lock:
                optimizer = discopt.DOptimizer(self.system.dsys, cost)
            optimizer.optimize_ic = False
            optimizer.descent_tolerance = 1e-3
            optimizer.first_method_iterations = 0
            # print Xref.shape, Uref.shape
            finished, X, U = optimizer.optimize(np.array(Xref_local), np.array(Uref_local),
                                                max_steps=90)
            if not finished:
                rospy.logwarn("Optimization failed!!!")
                break
            else:
                # print("lock status = %s"%self.lock.locked())
                with self.lock:
                # then we can get the new controller:
                    self.K, self.A, self.B = self.system.dsys.calc_feedback_controller(X, U,
                                                            return_linearization=True)
                    self.Uref = U
                    self.Xref = X
        return


    # this callback is responsible for obtaining the filtered results from the
    # controller program, and adding them to the local trajectory of filtered
    # measurements.  We must use locks to remain thread safe with the
    # optimization thread.
    def meascb(self, data):
        rospy.logdebug("receding_optimizer entered its measurement callback")
        if self.first_flag:
            self.base_time = data.header.stamp
            self.first_flag = False

        Xtmp = [1]*self.system.dsys.nX
        Xtmp[0] = data.xm
        Xtmp[1] = data.ym
        Xtmp[2] = data.xr
        Xtmp[3] = data.r
        Xtmp[4] = data.pxm
        Xtmp[5] = data.pym
        Xtmp[6] = data.vxr
        Xtmp[7] = data.vr

        with self.lock:
            self.Xfilt.append(Xtmp)
        return


    # this is the service handler:
    def control_service_handler(self, req):
        """
        Return the feedforward and feedback data for the trajectory base on the
        current optimization.
        """
        con = PlanarControlLaw()
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
        else:
            # use the index:
            if req.index > len(self.tref)-1:
                rospy.logerr("Requested index is too large!")
                return None
            index = req.index
        kkey = np.ravel(self.K[index]).tolist()
        uffkey = self.Uref[index]
        con.header.frame_id = "/optimization_frame"
        con.header.stamp = rospy.Time.now()
        con.uff = uffkey
        con.K.layout.dim.append(MultiArrayDimension())
        con.K.layout.dim.append(MultiArrayDimension())
        con.K.layout.dim[0].label = "height"
        con.K.layout.dim[0].size = 2
        con.K.layout.dim[0].stride = 16
        con.K.layout.dim[1].label = "width"
        con.K.layout.dim[1].size = 8
        con.K.layout.dim[1].stride = 8
        con.K.data = kkey
        return con



def main():
    """
    Run the main loop, by instatiating a System class, and then
    calling ros.spin
    """
    rospy.init_node('receding_optimizer', log_level=rospy.DEBUG)

    try:
        sim = RecedingOptimizer()
    except rospy.ROSInterruptException: pass
    rospy.spin()


if __name__=='__main__':
    main()
