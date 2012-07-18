// mass_marker.cpp
// Jarvis Schultz
// Summer 2012


//---------------------------------------------------------------------------
// Notes
// ---------------------------------------------------------------------------
// Just a simple node that subscribes to the position of the mass as
// determined by either a simulator, or an experimental estimator.  It
// then publishes markers that represent the mass location and the
// string.

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

//---------------------------------------------------------------------------
// Objects and Functions
//---------------------------------------------------------------------------
class MassMarker {

private:
    ros::NodeHandle n_;
    ros::Subscriber mass_sub;
    ros::Publisher marker_pub;
    visualization_msgs::Marker mass_marker;
    visualization_msgs::Marker string_marker;
    tf::TransformListener listener;


public:
    MassMarker() {
	// define publishers and subscribers
	ROS_DEBUG("Starting the Mass Marker node");
	mass_sub = n_.subscribe("mass_location", 1, &MassMarker::datacb, this);
	marker_pub = n_.advertise<visualization_msgs::Marker>("mass_marker", 1);

	// define the marker properties:
	mass_marker.ns = string_marker.ns = "markers";
	mass_marker.action = string_marker.action = visualization_msgs::Marker::ADD;
	int id = 0;
	if (ros::param::has("robot_index"))
	    ros::param::get("robot_index", id);
	mass_marker.id = id;
	string_marker.id = id+1;

	// visualization_properties and types
	mass_marker.type = visualization_msgs::Marker::SPHERE;
	mass_marker.scale.x = 0.06;
	mass_marker.scale.y = 0.06;
	mass_marker.scale.z = 0.06;
	mass_marker.color.r = 1.0f;
	mass_marker.color.g = 1.0f;
	mass_marker.color.b = 1.0f;
	mass_marker.color.a = 1.0f;
	mass_marker.lifetime = ros::Duration();

	string_marker.type = visualization_msgs::Marker::LINE_LIST;
	string_marker.scale.x = .01;
	string_marker.color.a = 1.0f;
	string_marker.points.resize(2);
	return;
    }

    void datacb(const geometry_msgs::PointStamped &p)
	{
	    ROS_DEBUG("Entered marker datacb");
	    // first publish the mass marker
	    mass_marker.pose.position = p.point;
	    mass_marker.header = p.header;
	    marker_pub.publish(mass_marker);

	    // now the string marker
	    string_marker.header = p.header;
	    string_marker.points[0] = p.point;
	    tf::StampedTransform trans;
	    geometry_msgs::TransformStamped ts;
	    geometry_msgs::Point pt;
	    try{
	    	listener.lookupTransform(
	    	    p.header.frame_id, "/robot_1/base_link",
	    	    p.header.stamp-ros::Duration(0.05), trans);
	    	tf::transformStampedTFToMsg(trans, ts);
	    	pt.x = ts.transform.translation.x;
	    	pt.y = ts.transform.translation.y;
	    	pt.z = ts.transform.translation.z;
	    	string_marker.points[1] = pt;
	    }
	    catch(tf::TransformException& ex){
	    	ROS_DEBUG(
	    	    "Error trying to lookupTransform: %s", ex.what());
	    	return;
	    }
	    marker_pub.publish(string_marker);
	    

	    return;
	}

};



//--------------------------------------------------------------------------
// Main
//--------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mass_marker");
  ros::NodeHandle n;

  MassMarker marker;

  ros::spin();

  return 0;
}
    
