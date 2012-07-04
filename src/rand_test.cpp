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
#include "boost/random/normal_distribution.hpp"
#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"

#include "puppeteer_msgs/RobotCommands.h"



/////////////
// GLOBALS //
/////////////
#define INT_FREQUENCY (200.0) // frequency we integrate kinematics at
#define WHEEL_DIA (0.07619999999999)
#define WIDTH (0.148/2.0)
#define PUB_FREQUENCY (30.0)
#define DEFAULT_DRIFT (0.0)
#define DEFAULT_NOISE (0.0)


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
    boost::variate_generator<boost::mt19937, boost::normal_distribution<> >* generator;


public:
    Simulator() {
	ROS_INFO("Starting up a robot simulator node");

	// define a timer to integrate the kinematics forward in time
	timer = n_.createTimer(ros::Duration(1/INT_FREQUENCY),
			       &Simulator::timercb, this);

	return;
    }

    ~Simulator() {
	delete generator;
    }

    template<class T>
    double gen_normal(T &generator)
	{
	    return generator();
	}
   
    void timercb(const ros::TimerEvent& e)
	{
	    static boost::variate_generator<boost::mt19937, boost::normal_distribution<> >
		tmp(boost::mt19937(time(0)),
			  boost::normal_distribution<>());
	    double d = 0.0;
	    if (ros::param::has("/simulator_noise"))
	    	ros::param::get("/simulator_noise", d);
	    else
	    	d = DEFAULT_DRIFT;

	    std::cout << "d*rand = " << d*gen_normal(tmp) << std::endl;

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

