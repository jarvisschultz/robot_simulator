#!/usr/bin/env python
"""
Jarvis Schultz
September 2013

This file simply generates reference configurations/ states for the suspended
mass system.

SUBSCRIPTIONS:
    - Time (start_time)

PUBLISHERS:

SERVICES:
    - PlanarSystemService (get_ref_config) (provider)
    - PlanarStateAbsTime (get_ref_state) (provider)

"""
import roslib; roslib.load_manifest('robot_simulator')
import rospy
import tf
from std_msgs.msg import Time
from puppeteer_msgs.msg import PlanarSystemConfig
from puppeteer_msgs.msg import PlanarSystemState
from puppeteer_msgs.srv import PlanarStateAbsTime
from puppeteer_msgs.srv import PlanarSystemService
from puppeteer_msgs.srv import PlanarSystemServiceRequest
import sys, os

## internal
import receding_planar_controller as rp


class PathGenerator:
    def __init__(self):
        args = sys.argv
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
        self.dt = self.tref[1] - self.tref[0]
        system.create_dsys(self.tref)
        (self.Xref, self.Uref) = system.dsys.build_trajectory(
             Q=self.Qref, p=self.pref, v=self.vref, u=self.uref, rho=self.rhoref)

        
        # define all publishers, subscribers, and services
        self.tstart_sub = rospy.Subscriber("start_time", Time, self.timecb)
        self.conf_serv = rospy.Service("get_ref_config", PlanarSystemService,
                                       self.ref_config_service_handler)
        self.state_serv = rospy.Service("get_ref_state", PlanarStateAbsTime,
                                        self.ref_state_service_handler)
        self.tbase = rospy.Time(0)
        return

    def timecb(self, time):
        """
        When the controller node publishes a new start time, this callback
        updates the local copy of the base ROS time
        """
        self.tbase = time.data
        return

    def ref_config_service_handler(self, req):
        """
        Return a reference configuration for the system by either
        index or time
        """
        config = PlanarSystemConfig()
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
        config.xm = self.Qref[index][0]
        config.ym = self.Qref[index][1]
        config.xr = self.Qref[index][2]
        config.r  = self.Qref[index][3]
        config.header.frame_id = "/optimization_frame"
        config.header.stamp = rospy.Time.now()
        time = self.tref[index]
        dt = self.dt
        length = len(self.tref)
        if index != len(self.tref)-1:
            utmp = self.Uref[index]
        else:
            utmp = self.Uref[-1]
        xtmp = self.Xref[index]
        return {'config' : config,
                'input' : utmp,
                'state' : xtmp,
                'dt' : dt,
                'length' : length,
                'time' : time,
                'index' : index}

    def ref_state_service_handler(self, req):
        tdesired = req.t
        # first check if the tbase has properly been set:
        if abs(self.tbase.to_sec()) < 0.01:
            rospy.logwarn("Path generator does not have a base time yet")
            return
        # since it has, let's manually call the other handler to fill out state reply:
        t = (tdesired-self.tbase).to_sec()
        req = PlanarSystemServiceRequest(t=t)
        resp = self.ref_config_service_handler(req)
        # now convert the response into a state message and reply with that:
        replystate = PlanarSystemState()
        replybool = False
        if resp is None:
            replybool = True
        else:
            replystate.header = resp['config'].header
            state = resp['state']
            replystate.xm  = state[0]
            replystate.ym  = state[1]
            replystate.xr  = state[2]
            replystate.r   = state[3]
            replystate.pxm = state[4]
            replystate.pym = state[5]
            replystate.vxr = state[6]
            replystate.vr  = state[7]
        return {'state' : replystate,
                'stop'  : replybool}




def main():
    rospy.init_node('config_generator', log_level=rospy.INFO)

    try:
        gen = PathGenerator()
    except rospy.ROSInterruptException: pass

    rospy.spin()



if __name__=='__main__':
    main()
