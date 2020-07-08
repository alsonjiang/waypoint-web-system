#include <ros/ros.h>

#include <waypoint_msgs/TotalWaypoint.h>
#include <waypoint_msgs/WaypointSequence.h>


bool callback_service(waypoint_msgs::TotalWaypoint::Request &req, waypoint_msgs::TotalWaypoint::Response &res)
{
	res.list.resize(4);
	res.list[0].location = "toilet";
	res.list[0].x = 0;
	res.list[0].y = 0;
	res.list[0].w = 0;
	res.list[0].z = 0;

	res.list[1].location = "kitchen";
	res.list[1].x = 1;
	res.list[1].y = 1;
	res.list[1].w = 1;
	res.list[1].z = 1;

	res.list[2].location = "balcony";
	res.list[2].x = 2;
	res.list[2].y = 2;
	res.list[2].w = 2;
	res.list[2].z = 2;

	res.list[3].location = "bedroom";
	res.list[3].x = 3;
	res.list[3].y = 3;
	res.list[3].w = 3;
	res.list[3].z = 3;

	return true;
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "service_node");
	ros::NodeHandle nh;
	ros::ServiceServer list_service = nh.advertiseService("/waypoint_list", callback_service);

	ros::spin();
}
