// Jarvis Schultz

// June 2012

///////////
// NOTES //
///////////
// This code provides a simulator to one of the diff-drive robots.  It
// subscribes to the serial commands, and integrates the kinematics
// forward in time.  It then publishes the results of the integration
// on a "vo" topic with a prescribed amount of noise.  We combine this
// with the urdf representing the robot, and we have a fully
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
#include <geometry_msgs/Point.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <angles/angles.h>
#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"

#include "puppeteer_msgs/RobotCommands.h"
#include "puppeteer_msgs/FullRobotState.h"



/////////////
// GLOBALS //
/////////////
#define INT_FREQUENCY (200.0) // frequency we integrate kinematics at
#define WHEEL_DIA (0.0761999999)
#define PULLEY_DIA (0.034924999)
#define WIDTH (0.148/2.0)
#define PUB_FREQUENCY (30.0)
#define TIMEOUT_FREQ (5.0) // frequency that we test for not receiving serial commands
#define DEFAULT_NOISE (0.0)
#define MAX_FLOATS (5) // maximum number of floats that we can send
		       // with one command
#define DEFAULT_STRING_LEN (1)
#define OCCLUSION_LIMS (30)

///////////////////////////
// OBJECTS AND FUNCTIONS //
///////////////////////////

class Simulator
{
private:
    ros::NodeHandle n_;
    ros::Subscriber sub;
    ros::Publisher pub, ser_pub, noise_free_pub;
    ros::Publisher string_pub_noise_free, string_pub;
    ros::Publisher state_pub, state_pub_noise_free;
    ros::Timer timer, pub_time, watchdog;
    // nav_msgs::Odometry pose_odom;
    Eigen::Vector3d pose;
    Eigen::Vector2d inputs, str_inputs, strings;
    int robot_index;
    bool running_flag;
    tf::TransformBroadcaster br;
    bool timeout;
    double *orig_len;
    geometry_msgs::Point Lengths, Lengths_nf;
    boost::array<double,36ul> kincov;
    

public:
    Simulator() {
	ROS_INFO("Starting up a robot simulator node\n");

	// define a subscriber for the serial commands
	timeout = false;
	sub = n_.subscribe("serial_commands", 100, &Simulator::datacb, this);

	// define a timer to integrate the kinematics forward in time
	timer = n_.createTimer(ros::Duration(1/INT_FREQUENCY),
			       &Simulator::timercb, this);

	// create a timer for checking for serial timeouts
	watchdog = n_.createTimer(ros::Duration(1/TIMEOUT_FREQ),
				  &Simulator::watchdogcb, this);

	// create another timer to publish the results state of the
	// robot as an Odometry message and as a tf
	pub_time = n_.createTimer(ros::Duration(1/PUB_FREQUENCY),
				  &Simulator::publishcb, this);
	pub = n_.advertise<nav_msgs::Odometry>("vo", 100);
	noise_free_pub = n_.advertise<nav_msgs::Odometry>("vo_noise_free", 100);
	string_pub = n_.advertise<geometry_msgs::Point>("string_lengths", 100);
	string_pub_noise_free =
	    n_.advertise<geometry_msgs::Point>("string_lengths_noise_free",100);
	
	// create a publisher for telling other nodes what commands we
	// sent to the robot
	ser_pub = n_.advertise<geometry_msgs::PointStamped>
	    ("serviced_values", 100);

	// final set of publishers for sending out the complete robot state:
	state_pub = n_.advertise<puppeteer_msgs::FullRobotState>
	    ("robot_state", 100);
	state_pub_noise_free = n_.advertise<puppeteer_msgs::FullRobotState>
	    ("robot_state_noise_free", 100);	    
	
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
	orig_len = new double[2];
	orig_len[0] = DEFAULT_STRING_LEN;
	orig_len[1] = DEFAULT_STRING_LEN;

	// set covariance values:
	double kin_cov_dist = 0.5;	// in meters^2
	double kin_cov_ori = 100.0;	// radians^2
	boost::array<double,36ul> tmp = {{kin_cov_dist, 0, 0, 0, 0, 0,
					  0, kin_cov_dist, 0, 0, 0, 0,
					  0, 0,        99999, 0, 0, 0,
					  0, 0, 0,        99999, 0, 0,
					  0, 0, 0, 0,        99999, 0,
					  0, 0, 0, 0, 0,  kin_cov_ori}};
	kincov = tmp;

	
	return;
    }

    ~Simulator() {
	delete orig_len;
    }
    


    template<class T>
    double gen_normal(T &generator)
	{
	    return generator();
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

	    // integrate the string heights:
	    strings(0) += str_inputs(0) * dt;
	    strings(1) += str_inputs(1) * dt;
	    return;
	}


    void publishcb(const ros::TimerEvent& e)
	{
	    static boost::variate_generator<boost::mt19937, boost::normal_distribution<> >
		noise(boost::mt19937(time(0)),
		    boost::normal_distribution<>());
	    // let's make a copy of pose, and then add some noise to it:
	    Eigen::Vector3d c = pose;

	    // now we need to add noise to the state
	    double d = 0.0;
	    if (ros::param::has("/simulator_noise"))
	    	ros::param::getCached("/simulator_noise", d);
	    else
	    	d = DEFAULT_NOISE;
	    Eigen::Vector3d drift;
	    drift << d*gen_normal(noise),d*gen_normal(noise),10*d*gen_normal(noise);
	    c += drift;

	    // build and publish the odometry message:
	    nav_msgs::Odometry odom, odom_nf;
	    std::stringstream ss;
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

	    // to simulate the effects of occlusions, let's
	    // occasionally publish a pose with a huge covariance
	    static unsigned int occlusion_cnt = 0;
	    occlusion_cnt++;
	    if (!(occlusion_cnt%OCCLUSION_LIMS))
	    {
		boost::array<double,36ul> tmpcov;
		for (int i=0; i<36; i++)
		    tmpcov[i] = kincov[i]*1000.0;
		odom.pose.covariance = tmpcov;
	    }
	    else
		odom.pose.covariance = kincov;
	    
	    pub.publish(odom);

	    // now publish the noise-free version:
	    c = pose;
	    odom_nf.header.stamp = odom.header.stamp;
	    odom_nf.header.frame_id = odom.header.frame_id;
	    odom_nf.child_frame_id = odom.child_frame_id;
	    odom_nf.pose.pose.position.x = c(0);
	    odom_nf.pose.pose.position.y = -c(1);
	    odom_nf.pose.pose.position.z = 0;
	    theta = -c(2);
	    quat =
		tf::createQuaternionMsgFromYaw(angles::normalize_angle(theta));
	    odom_nf.pose.pose.orientation = quat;
	    noise_free_pub.publish(odom_nf);
	    
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

	    ROS_DEBUG("Sending transform for output of simulator");
	    br.sendTransform(trans);

	    // now we can process and send the message for the string lengths:
	    send_string_states(d);

	    // send complete robot states
	    send_robot_states(odom, odom_nf);

	    return;
	}
	

    void datacb(const puppeteer_msgs::RobotCommands& c)
	{
	    ROS_DEBUG("Serial request received");
	    // reset watchdog timer:
	    timeout = false;
	    char type = c.type;
	    float vals[MAX_FLOATS];
	    memset(vals, 0, sizeof(vals));

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
		set_string_lengths(0.0, 0.0);
		break;
	    case 'q': // STOP
		set_inputs(0.0, 0.0);
		set_winch_inputs(0.0, 0.0);
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
		set_winch_inputs(PULLEY_DIA/2.0*c.v_top,
				 PULLEY_DIA/2.0*c.v_top);
		vals[0] = c.v_left;
		vals[1] = c.v_right;
		vals[2] = c.v_top;
		break;
	    case 'd': // EXT_SPEED
		set_inputs(c.v_robot, c.w_robot);
		set_winch_inputs(c.rdot, c.rdot);
		vals[0] = inputs(0);
		vals[1] = inputs(1);
		vals[2] = str_inputs(0);
		break;
	    case 'n': // MOT_SPEED_FULL
		// convert motor speeds to v and w:
		set_inputs(WHEEL_DIA*(c.v_left+c.v_right)/4.0,
			   WHEEL_DIA*(c.v_right-c.v_left)/(4.0*WIDTH));
		set_winch_inputs(PULLEY_DIA/2.0*c.v_top_left,
				 PULLEY_DIA/2.0*c.v_top_right);				
		vals[0] = c.v_left;
		vals[1] = c.v_right;
		vals[2] = c.v_top_left;
		break;
	    case 'i': // EXT_SPEED_FULL
		set_inputs(c.v_robot, c.w_robot);
		set_winch_inputs(c.rdot_left, c.rdot_right);
		vals[0] = inputs(0);
		vals[1] = inputs(1);
		vals[2] = str_inputs(0);
		break;
	    case 'a': // SET_CONFIG_FULL
		set_robot_state(c.x, c.y, c.th);
		set_string_lengths(c.height_left, c.height_right);
		break;
	    case 's': // SET_DEF_SPEED
		ROS_DEBUG("Cannot set default speed yet");
		break;
	    case 'l': // SET_POSE
		set_robot_state(c.x, c.y, c.th);
		break;
	    case 'b': // SET_HEIGHT
		set_string_lengths(c.height_left, c.height_right);
		break;
	    case 'w': // POSE_REQ
		ROS_DEBUG("No need to request information from robot simulator");
		break;
	    case 'e': // SPEED_REQ
		ROS_DEBUG("No need to request information from robot simulator");
		break;
	    }

	    // now let's publish the serviced_values topic
	    geometry_msgs::PointStamped cmd;
	    cmd.header.frame_id = type;
	    cmd.header.stamp = ros::Time::now();
	    cmd.point.x = vals[0];
	    cmd.point.y = vals[1];
	    cmd.point.z = vals[2];
	    ser_pub.publish(cmd);
	    
	    return;
	}


    void watchdogcb(const ros::TimerEvent& e)
	{
	    if (fabs(inputs(0)) > 0.005 || fabs(inputs(1)) > 0.005)
	    {
		if (timeout)
		{
		    set_inputs(0,0);
		    set_winch_inputs(0,0);
		    ROS_ERROR("Robot %d timeout!", robot_index);
		}
	    }
	    timeout = true;
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

    // util function for setting the internal values for the
    // translational velocities of each of the robot's winch motors.
    void set_winch_inputs(const float vleft, const float vright)
	{
	    str_inputs(0) = vleft;
	    str_inputs(1) = vright;
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

    // util function for setting the robot's internal representation
    // of the string lengths.
    void set_string_lengths(const float length_left, const float length_right)
	{
	    strings(0) = 0;
	    strings(1) = 0;
	    orig_len[0] = length_left;
	    orig_len[1] = length_right;
	    return;
	}
    

    // this function simply converts the string heights that we have
    // been integrating, and it publishes them as a ROS message that
    // contains the LENGTHS of the strings.
    void send_string_states(const double d)
	{
	    static boost::variate_generator<boost::mt19937, boost::normal_distribution<> >
		noise(boost::mt19937(time(0)),
		      boost::normal_distribution<>());
	    Eigen::Vector2d s = strings;
	    Eigen::Vector2d drift;
	    drift << d*gen_normal(noise),d*gen_normal(noise);
	    s += drift;

	    s(0) += orig_len[0];
	    s(1) += orig_len[1];

	    if (s(0) < 0 || s(1) < 0)
	    {
		ROS_ERROR("Simulator String Length Too Short!");
		exit(1);
	    }

	    Lengths.x = s(0);
	    Lengths.y = s(1);
	    string_pub.publish(Lengths);

	    s -= drift;
	    Lengths_nf.x = s(0);
	    Lengths_nf.y = s(1);
	    string_pub_noise_free.publish(Lengths_nf);
	    
	    return;
	}

    // this function is used for sending the complete state of the
    // robot as a puppeteer_msgs/FullRobotState message in both a
    // noisy, and noise free version.
    void send_robot_states(nav_msgs::Odometry o, nav_msgs::Odometry o_nf)
	{
	    puppeteer_msgs::FullRobotState st;
	    st.pose.header = o.header;
	    st.pose.pose = o.pose.pose;
	    st.left = Lengths.x;
	    st.right = Lengths.y;
	    state_pub.publish(st);

	    st.pose.header = o_nf.header;
	    st.pose.pose = o_nf.pose.pose;
	    st.left = Lengths_nf.x;
	    st.right = Lengths_nf.y;
	    state_pub_noise_free.publish(st);
	    
	    return;
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

    Simulator simul;
  
    ros::spin();
  
    return 0;
}

