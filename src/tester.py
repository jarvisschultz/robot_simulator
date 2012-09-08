#!/usr/bin/env python
import roslib; roslib.load_manifest('robot_simulator')
import rospy
from puppeteer_msgs.msg import FullRobotState
from puppeteer_msgs.msg import PlanarSystemConfig
from puppeteer_msgs.msg import RobotCommands
from puppeteer_msgs.srv import PlanarSystemService


def main():
    rospy.init_node('service_tester')
    # rospy.wait_for_service('get_ref_config')
    srv = rospy.ServiceProxy('/get_ref_config', PlanarSystemService)

    try:
        resp1 = srv(t=8.0)
        print "Response:"
        print resp1.config.xm, resp1.config.ym, resp1.config.xr, resp1.config.r
    except rospy.ServiceException, e:
        print "Service did not process request: %s"%str(e)



if __name__=='__main__':
    main()
