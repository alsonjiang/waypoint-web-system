#include <ros/ros.h>

#include <waypoint_msgs/TotalWaypoint.h>
#include <waypoint_msgs/WaypointSequence.h>

ros::ServiceClient client;


int main (int argc, char **argv)
{
	ros::init(argc, argv, "trigger");
	ros::NodeHandle nh;
	client = nh.serviceClient<waypoint_msgs::WaypointSequence>("/waypoint_sequence");
	
	waypoint_msgs::WaypointSequence srvmsg;
	srvmsg.request.sequence.resize(4);
	srvmsg.request.sequence[0].location = "home";
	srvmsg.request.sequence[1].location = "yomom";
	srvmsg.request.sequence[2].location = "bus";
	srvmsg.request.sequence[3].location = "school";

	srvmsg.request.sequence[0].task = "punch";
	srvmsg.request.sequence[1].task = "bye";
	srvmsg.request.sequence[2].task = "sleep";
	srvmsg.request.sequence[3].task = "hello";
	client.call(srvmsg);

	
	ros::spin();
}
