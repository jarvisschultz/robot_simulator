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

//---------------------------------------------------------------------------
// Objects and Functions
//---------------------------------------------------------------------------
class MassMarker {

private:
    ros::NodeHandle n_;
    ros::Subscriber mass_sub;
    ros::Publisher marker_pub;
    visualization_msgs::Marker mass_marker;


public:
    MassMarker() {
	// define publishers and subscribers
	ROS_DEBUG("Starting the Mass Marker node");
	mass_sub = n_.subscribe("mass_location", 1, &MassMarker::datacb, this);
	marker_pub = n_.advertise<visualization_msgs::Marker>("mass_marker", 1);

	// define the marker properties:
	mass_marker.ns = "markers";
	int id = 0;
	if (ros::param::has("robot_index"))
	    ros::param::get("robot_index", id);
	mass_marker.id = id;
	mass_marker.type = visualization_msgs::Marker::SPHERE;
	mass_marker.scale.x = 0.06;
	mass_marker.scale.y = 0.06;
	mass_marker.scale.z = 0.06;
	mass_marker.color.r = 1.0f;
	mass_marker.color.g = 1.0f;
	mass_marker.color.b = 1.0f;
	mass_marker.color.a = 1.0f;
	mass_marker.lifetime = ros::Duration();

	return;
    }

    void datacb(const geometry_msgs::PointStamped &p)
	{
	    ROS_DEBUG("Entered marker datacb");
	    mass_marker.pose.position = p.point;
	    mass_marker.header = p.header;
	    marker_pub.publish(mass_marker);

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
    
