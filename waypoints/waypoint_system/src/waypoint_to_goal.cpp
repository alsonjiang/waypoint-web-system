#include <ros/ros.h>

#include <waypoint_msgs/WaypointsList.h>
#include <waypoint_msgs/WaypointSequence.h>
#include <waypoint_msgs/PathTaskArray.h>
#include <std_msgs/String.h>

//Sequence user given waypoints
class Sequencer
{
    private:
    ros::Publisher path_publisher;//Publisher for sequenced waypoints
    ros::ServiceServer sequence_server;//Server to receive user given waypoints
    ros::ServiceClient list_client;//Client to call list of all waypoints
    waypoint_msgs::PathTaskArray path_msg;//Store sequenced waypoints for publish

    //Function for sequencing user given waypoints
    //Requires user given waypoints and list of all waypoints as parameters
    waypoint_msgs::PathTaskArray waypoint_sequencer(waypoint_msgs::WaypointSequence::Request order, waypoint_msgs::WaypointsList::Response table)
    {
        ROS_INFO("sequencing");
        waypoint_msgs::PathTaskArray path;//Store sequenced waypoints
        int sequence_amt = order.sequence.size();//Array size of user given waypoints
        path.poses.resize(sequence_amt);//Equate the size of the array to be same as the user given waypoints
        //Go through all user given waypoints
        for(int i=0; i<sequence_amt; i++)
        {
            int list_amt = table.ID.size();//Array size of the list of the total waypoints
            //Go through all the waypoints in the list
            for(int j=0; j<list_amt; j++)
            {
                //Check if the location name correspond between the user given waypoints and the waypoints in the list
                if(order.sequence[i].location == table.ID[j].name)
                {
                    //store sequenced waypoints
                    path.poses[i].pose.position.x = stoi(table.ID[j].pose.x);
                    path.poses[i].pose.position.y = stoi(table.ID[j].pose.y);
                    path.poses[i].pose.orientation.z = stoi(table.ID[j].pose.z);
                    path.poses[i].pose.orientation.w = stoi(table.ID[j].pose.w);

                    path.poses[i].task = order.sequence[i].task;
                }
            }
        }
        return path;
    }

    public:
    Sequencer(ros::NodeHandle *nh)
    {
        path_publisher = nh->advertise<waypoint_msgs::PathTaskArray>("/waypoints", 10);
        sequence_server = nh->advertiseService("/waypoint_sequence", &Sequencer::callback_sequence, this);
        list_client = nh->serviceClient<waypoint_msgs::WaypointsList>("/web_service/retrieve_all_location");
    }

    //recieve user given waypoints
    bool callback_sequence(waypoint_msgs::WaypointSequence::Request &req, waypoint_msgs::WaypointSequence::Response &res)
    {
        ROS_INFO("callback");
        waypoint_msgs::WaypointsList msgsrv;
        //call for list of all waypoints
	    if (list_client.call(msgsrv))
		{
            path_msg = waypoint_sequencer(req, msgsrv.response);//sequence user given waypoints
            path_publisher.publish(path_msg);//publish sequenced waypoints
            res.success = "true";
		}
		else
		{
			ROS_ERROR("Failed to obtain waypoint list");
            res.success = "false";
		}
        return true;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_to_goal");
    ros::NodeHandle nh;
    Sequencer wg = Sequencer(&nh);
    ros::spin();
}