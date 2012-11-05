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
    -

SERVICES:
    - PlanarSystemService (get_receding_config)
    - Float32MultiArray (get_receding_controller)

"""

## define all imports:
import roslib; roslib.load_manifest('robot_simulator')
import rospy
import tf
from puppeteer_msgs.msg import FullRobotState
from puppeteer_msgs.msg import PlanarSystemConfig
from puppeteer_msgs.msg import RobotCommands
from puppeteer_msgs.srv import PlanarSystemService
from puppeteer_msgs.srv import PlanarSystemServiceRequest
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
# import receding_planar_controller.py
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
