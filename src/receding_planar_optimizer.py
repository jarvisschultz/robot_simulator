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
    - PlanarSystemConfig (filt_config)

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
        self.filt_sub = rospy.Subscriber("filt_config", PlanarSystemConfig,
                                         self.meascb)

        # define miscellaneous variables related to the receding horizon controller:
        self.system.create_dsys(np.array(self.tref))
        # define the cost weights:
        Qcost = np.diag([10000, 10000, 1, 1, 1000, 1000, 300, 300])
        Rcost = np.diag([1, 1])
        # Xtmp = np.array(copy.deepcopy(self.Xref))
        # Utmp = np.array(copy.deepcopy(self.Uref))
        cost = discopt.DCost(np.array(self.Xref), np.array(self.Uref), Qcost, Rcost)
        optimizer = discopt.DOptimizer(self.system.dsys, cost)


        # manually open up another thread that repeatedly performs the full
        # discrete optimization as fast as possible:
        optimizer.optimize_ic = False
        optimizer.first_method_iterations = 0
        finished, X, U = optimizer.optimize(self.Xref, self.Uref, max_steps=90)
        if not finished:
            rospy.logwarn("Optimization failed!!!")
        else:
            # then we can get the new controller:
            self.K, self.A, self.B = self.system.dsys.calc_feedback_controller(X, U,
                                                        return_linearization=True)
            self.Uref_new = U
            self.Xref_new = X


        return


    # this callback is responsible for obtaining the filtered results from the
    # controller program, and adding them to the local trajectory of filtered
    # measurements.  We must use locks to remain thread safe with the
    # optimization thread.
    def meascb(self, data):
        rospy.logdebug("receding_optimizer entered its measurement callback")

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
    rospy.init_node('receding_optimizer', log_level=rospy.INFO)

    try:
        sim = RecedingOptimizer()
    except rospy.ROSInterruptException: pass
    rospy.spin()


if __name__=='__main__':
    main()
