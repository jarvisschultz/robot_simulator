// Jarvis Schultz

// June 2012

///////////
// NOTES //
///////////
// This code provides a simulator to one of the diff-drive robots.  It
// subscribes to the serial commands, and integrates the kinematics
// forward in time.  It then publishes the results of the integration
// on a "vo" topic with a prescribed amount of noise.  We combine this
// wiht the urdf representing the robot, and we have a fully
// interactive model of the robot.

//////////////
// INCLUDES //
//////////////
#include <vector>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <algorithm>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <puppeteer_msgs/PointPlus.h>
#include <puppeteer_msgs/Robots.h>
#include <geometry_msgs/Point.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "puppeteer_msgs/RobotCommands.h"

/////////////
// GLOBALS //
/////////////
#define INT_FREQUENCY (200.0); // frequency we integrate kinematics at


///////////////////////////
// OBJECTS AND FUNCTIONS //
///////////////////////////

class Simulator
{
private:
    ros::NodeHandle n_;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Timer timer;
    nav_msgs::Odometry pose;
    Eigen::Vector2d in;
    unsigned int robot_index;

   


public:
    Simulator() {
	ROS_INFO("Starting up a robot simulator node");

	// define a subscriber for the serial commands
	sub = n_.subscribe("serial_commands", 100, &Simulator::datacb, this);

	// define a timer to integrate the kinematics forward in time
	timer = n_.createTimer(ros::Duration(1/INT_FREQUENCY),
			       &Coordinator::timercb, this);

	// define a publisher for sending out the current pose of the robot:
	if (ros::param::has("robot_index"))	
	    ros::param::get("robot_index", robot_index);
	else
	{
	    ROS_WARN("Robot index not set, using a default");
	    ros::param::set("robot_index", 1);
	    robot_index = 1;
	}

	

	return;
    }

    
    // in this function, we take in two angles, and using one as the
    // reference, we keep adding and subtracting 2pi from the other to
    // find the mininmum angle between the two:
    void angle_correction(double& a, double& ref)
	{
	    while ((a-ref) > M_PI) a -= 2*M_PI;
	    while ((a-ref) < -M_PI) a += 2*M_PI;
	}


    
    double clamp_angle(double theta)
	{
	    double th = theta;
	    while(th > M_PI)
		th -= 2.0*M_PI;
	    while(th <= -M_PI)
		th += 2.0*M_PI;
	    return th;
	}


}; // END Simulator class






//////////
// MAIN //
//////////
int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_simulator");

    // // turn on debugging
    // log4cxx::LoggerPtr my_logger =
    // log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
    // my_logger->setLevel(
    // ros::console::g_level_lookup[ros::console::levels::Debug]);

    ros::NodeHandle n;

    ROS_INFO("Starting Robot Simulator Node...\n");
    Simulator simul;
  
    ros::spin();
  
    return 0;
}

