// planar_coordinator.cpp
// Jarvis Schultz
// Fall 2012

//---------------------------------------------------------------------------
// Notes
// ---------------------------------------------------------------------------
// This node is used for coordinating the output of two of the object
// tracking nodes. One for tracking the robot, and one for tracking
// the suspended mass.  This node subscribes to the outputs of each
// node, and publishes the results of both incoming PointPlus messages
// as a puppeteer_msgs/PlanarSystemConfig message.  It is also
// responsible for handling all of the logic related to the
// /operating_condition and performing the calibration.


///////////////////////////////////////////////////////////////////////////
// // SUBSCRIPTIONS:							 //
// //	- robot_kinect_position (puppeteer_msgs::PointPlus)		 //
// //	- object1_position (puppeteer_msgs::PointPlus)			 //
// //									 //
// // PUBLISHERS:							 //
// //	- meas_config (puppeteer_msgs::PlanarSystemConfig)		 //
// //									 //
// // SERVICE CALLS:							 //
// //	- get_ref_config (puppeteer_msgs::PlanarSystemService)		 //
///////////////////////////////////////////////////////////////////////////

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <puppeteer_msgs/PointPlus.h>
#include <puppeteer_msgs/PlanarSystemConfig.h>
#include <geometry_msgs/Point.h>


//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------
#define NUM_CALIBRATES (30)
#define NUM_EKF_INITS (3)
#define ROBOT_CIRCUMFERENCE (57.5) // centimeters
#define DEFAULT_RADIUS (ROBOT_CIRCUMFERENCE/M_PI/2.0/100.) // meters


//---------------------------------------------------------------------------
// Objects and Functions
//---------------------------------------------------------------------------
using namespace message_filters;
using namespace puppeteer_msgs;


class PlanarCoordinator {

private:
    ros::NodeHandle node_;
    ros::Publisher config_pub;
    message_filters::Subscriber<PointPlus> robot_kinect_sub;
    message_filters::Subscriber<PointPlus> mass_kinect_sub;
    message_filters::TimeSynchronizer<PointPlus, PointPlus> * sync;
    ros::Time t_now_timer, t_last_timer;
    double robot_start_ori, robot_radius;


public:
    PlanarCoordinator () {
	ROS_DEBUG("Instantiating a PlanarCoordinator Class");
	// define subscribers, synchronizer, and the corresponding
	// callback:

	robot_kinect_sub.subscribe(node_, "robot_kinect_position", 10);
	mass_kinect_sub.subscribe(node_, "object1_position", 10);
	sync = new message_filters::TimeSynchronizer<PointPlus, PointPlus>
	    (robot_kinect_sub, mass_kinect_sub, 10);
	sync->registerCallback(boost::bind(
				   &PlanarCoordinator::callback, this, _1, _2));
	// define publisher
	config_pub = node_.advertise<PlanarSystemConfig> ("meas_config", 100);

	
	return;
    }

    ~PlanarCoordinator() {
	delete sync;
	return;
    }

    void callback(const PointPlusConstPtr& rob_pt, const PointPlusConstPtr& mass_pt)
	{
	    ROS_DEBUG("Synchronized Callback triggered");
	    // printf("robot position: x=%f y=%f z=%f\r\n", rob_pt->x, rob_pt->y, rob_pt->z);
	    // printf("mass position: x=%f y=%f z=%f\r\n", mass_pt->x, mass_pt->y, mass_pt->z);
	    PlanarSystemConfig cfg;
	    cfg.header.stamp = rob_pt->header.stamp;
	    cfg.header.frame_id = rob_pt->header.frame_id;
	    config_pub.publish(cfg);
	    return;
	}
    

}; // End of the PlanarCoordinator() Class



//--------------------------------------------------------------------------
// Main
//--------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "planar_system_coordinator");

    log4cxx::LoggerPtr my_logger =
    	log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
    my_logger->setLevel(
	ros::console::g_level_lookup[ros::console::levels::Debug]);

    ros::NodeHandle node;

    PlanarCoordinator planar_coordinator;
    ros::spin();

    return 0;
}



