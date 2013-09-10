#!/usr/bin/env python
"""
Jarvis Schultz
September 2013

This file simply generates reference configurations/ states for the suspended
mass system.

SUBSCRIPTIONS:

PUBLISHERS:
    - Path (mass_ref_path)

SERVICES:
    - PlanarSystemService (get_ref_config) (provider)

"""
import roslib; roslib.load_manifest('robot_simulator')
import rospy
import tf
from puppeteer_msgs.msg import PlanarSystemConfig
from puppeteer_msgs.srv import PlanarSystemService
from puppeteer_msgs.msg import PlanarCovariance
from puppeteer_msgs.msg import PlanarControlLaw
from puppeteer_msgs.srv import PlanarControlLawService
from puppeteer_msgs.srv import PlanarControlLawServiceRequest
from puppeteer_msgs.msg import PlanarSystemState
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from math import sin, cos, floor
from math import pi as mpi
import numpy as np
import scipy as sp
from scipy.interpolate import interp1d as spline

# local imports:
import receding_planar_controller as rp



class PathGenerator:
    def __init__(self):
        rospy.loginfo("Mass path generator started")
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

        # load traj
        system = rp.MassSystem2D()
        (self.tref, self.Qref, self.pref, self.vref, self.uref, self.rhoref) = \
            rp.trep.load_trajectory(fname, system.sys)

        # define all publishers, subscribers, and services
        self.conf_serv = rospy.Service("get_ref_config", PlanarSystemService,
            self.ref_config_service_handler)

        

        
        




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




def main():
    rospy.init_node('config_generator', log_level=rospy.INFO)

    try:
        gen = PathGenerator()
    except rospy.ROSInterruptException: pass

    rospy.spin()



if __name__=='__main__':
    main()
