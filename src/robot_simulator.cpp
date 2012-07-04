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
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <angles/angles.h>

#include "puppeteer_msgs/RobotCommands.h"



/////////////
// GLOBALS //
/////////////
#define INT_FREQUENCY (200.0) // frequency we integrate kinematics at
#define WHEEL_DIA (0.07619999999999)
#define WIDTH (0.148/2.0)
#define PUB_FREQUENCY (30.0)



///////////////////////////
// OBJECTS AND FUNCTIONS //
///////////////////////////

class Simulator
{
private:
    ros::NodeHandle n_;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Timer timer, pub_time;
    // nav_msgs::Odometry pose_odom;
    Eigen::Vector3d pose;
    Eigen::Vector2d inputs;
    int robot_index;
    bool running_flag;
    tf::TransformBroadcaster br;
   


public:
    Simulator() {
	ROS_INFO("Starting up a robot simulator node");

	// define a subscriber for the serial commands
	// sub = n_.subscribe("/serial_commands", 100, &Simulator::datacb, this);
	sub = n_.subscribe("serial_commands", 100, &Simulator::datacb, this);

	// define a timer to integrate the kinematics forward in time
	timer = n_.createTimer(ros::Duration(1/INT_FREQUENCY),
			       &Simulator::timercb, this);

	// create another timer to publish the results state of the
	// robot as an Odometry message and as a tf
	pub_time = n_.createTimer(ros::Duration(1/PUB_FREQUENCY),
				  &Simulator::publishcb, this);
	pub = n_.advertise<nav_msgs::Odometry>("vo", 100);
	
	// define a publisher for sending out the current pose of the robot:
	if (ros::param::has("robot_index"))	
	    ros::param::get("robot_index", robot_index);
	else
	{
	    ROS_WARN("Robot index not set, using a default");
	    ros::param::set("robot_index", 1);
	    robot_index = 1;
	}  
		
	// initialize vars that need it:
	running_flag = false;
	

	return;
    }


    void timercb(const ros::TimerEvent& e)
	{
	    ROS_DEBUG("Simulator timer callback triggered... integrate kinematics");
	    static ros::Time tcall = ros::Time::now();

	    double dt = (ros::Time::now()-tcall).toSec();
	    
	    pose(0) += cos(pose(2)) * inputs(0) * dt;
	    pose(1) += sin(pose(2)) * inputs(0) * dt;
	    pose(2) += inputs(1) * dt;

	    // correct angle:
	    pose(2) = angles::normalize_angle(pose(2));
	    
	    tcall = ros::Time::now();
	    return;
	}


    void publishcb(const ros::TimerEvent& e)
	{
	    Eigen::Vector3d c = pose;
	    nav_msgs::Odometry odom;

	    std::string ns = ros::names::clean(ros::this_node::getNamespace());
	    std::stringstream ss;
	    // if (ns.size() > 1)
	    // 	// ss << "base_footprint_kinect_" << ns.substr(1);
	    // 	ss << "robot_" << ns.substr(1) << "/base_footprint_kinect";	    
	    // else 
	    // 	ss << "base_footprint_kinect";
	    ss << "base_footprint_kinect";
	    
	    odom.header.stamp = ros::Time::now();
	    odom.header.frame_id = "/map";
	    odom.child_frame_id = ss.str();
	    odom.pose.pose.position.x = c(0);
	    odom.pose.pose.position.y = -c(1);
	    odom.pose.pose.position.z = 0;

	    double theta = -c(2);
	    geometry_msgs::Quaternion quat =
		tf::createQuaternionMsgFromYaw(angles::normalize_angle(theta));
	    odom.pose.pose.orientation = quat;

	    ROS_DEBUG("Simulator publishing on vo topic...");
	    pub.publish(odom);

	    // now let's send out the corresponding transform as well
	    geometry_msgs::TransformStamped trans;
	    tf::Quaternion q1, q2;
	    q1 = tf::createQuaternionFromYaw(angles::normalize_angle(theta));
	    q2 = tf::Quaternion(1.0,0,0,0);
	    q1 = q1*q2;
	    tf::quaternionTFToMsg(q1, quat);

	    trans.header.stamp = ros::Time::now();
	    trans.header.frame_id = odom.header.frame_id;
	    trans.child_frame_id = odom.child_frame_id;
	    trans.transform.translation.x = odom.pose.pose.position.x;
	    trans.transform.translation.y = odom.pose.pose.position.y;
	    trans.transform.translation.z = odom.pose.pose.position.z;
	    trans.transform.rotation = quat;

	    ROS_DEBUG("Sending transform for output of estimator node");
	    br.sendTransform(trans);

	    return;
	}
	

    void datacb(const puppeteer_msgs::RobotCommands& c)
	{
	    ROS_DEBUG("Serial request received");
	    char type = c.type;

	    if (c.robot_index != robot_index && c.robot_index != 9)
	    {
		ROS_DEBUG("Not the correct robot_index "
			  "(should be %d, actually is %d)!",
			  robot_index, c.robot_index);
		return;
	    }

	    switch(type)
	    {
	    case 'p': // REF_POSE
		ROS_DEBUG("Pose control not yet implemented in sim");
		break;
	    case 'r': // RESET
		set_inputs(0.0, 0.0);
		set_robot_state(0.0, 0.0, 0.0);
		break;
	    case 'q': // STOP
		set_inputs(0.0, 0.0);
		break;
	    case 'm': // START
		running_flag = !running_flag;
		ROS_DEBUG("Received start command: flag = %s!",
			  running_flag ? "True" : "False");
		break;
	    case 'h': // MOT_SPEED
		// convert motor speeds to v and w:
		set_inputs(WHEEL_DIA*(c.v_left+c.v_right)/4.0,
			   WHEEL_DIA*(c.v_right-c.v_left)/(4.0*WIDTH));
		break;
	    case 'd': // EXT_SPEED
		set_inputs(c.v_robot, c.w_robot);
		break;
	    case 'n': // MOT_SPEED_FULL
		// convert motor speeds to v and w:
		set_inputs(WHEEL_DIA*(c.v_left+c.v_right)/4.0,
			   WHEEL_DIA*(c.v_right-c.v_left)/(4.0*WIDTH));
		break;
	    case 'i': // EXT_SPEED_FULL
		set_inputs(c.v_robot, c.w_robot);
		break;
	    case 'a': // SET_CONFIG_FULL
		set_robot_state(c.x, c.y, c.th);
		break;
	    case 's': // SET_DEF_SPEED
		ROS_DEBUG("Cannot set default speed yet");
		break;
	    case 'l': // SET_POSE
		set_robot_state(c.x, c.y, c.th);
		break;
	    case 'b': // SET_HEIGHT
		ROS_DEBUG("Cannot set the height of the string yet");
		break;
	    case 'w': // POSE_REQ
		ROS_DEBUG("No need to request information from robot simulator");
		break;
	    case 'e': // SPEED_REQ
		ROS_DEBUG("No need to request information from robot simulator");
		break;
	    }
	    
	    return;
	}


    // util function for setting the current values of the
    // translational and rotational velocity of the simulator
    void set_inputs(const float v, const float w)
	{
	    inputs(0) = v;
	    inputs(1) = w;
	    return;
	}


    // this is a util function for setting the simulator's current
    // state in its odom configuration:
    void set_robot_state(const float x, const float y, const float th)
	{
	    pose(0) = x;
	    pose(1) = y;
	    pose(2) = angles::normalize_angle(th);
	    
	    return;
	}


    // util function for converting the pose to an odometry message
    nav_msgs::Odometry convert_pose_to_odom(const Eigen::Vector3d& p)
	{
	    nav_msgs::Odometry odom;


	    return odom;
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

