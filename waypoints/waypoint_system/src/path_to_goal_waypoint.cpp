#include <ros/ros.h>
#include <iostream>

#include <actionlib/client/simple_action_client.h>

#include <waypoint_msgs/WaypointPauseTiming.h>
#include <waypoint_msgs/PathTaskArray.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <move_base_msgs/MoveBaseAction.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

waypoint_msgs::PathTaskArray storeGoals;
int amtOfGoals, currentGoal, pause_time[10], waypoint_num[10], pause_num=0;
bool loop_enabled=false;

void hello(void)
{
	ROS_INFO("HELLO");
}

void punch(void)
{
	ROS_INFO("PUNCH");
}

void bye(void)
{
	ROS_INFO("BYE");
}

void sleep(void)
{
	ROS_INFO("SLEEP");
}

void task(void)
{
	if(storeGoals.poses[currentGoal-1].task == "hello")
		hello();
	else if(storeGoals.poses[currentGoal-1].task == "punch")
		punch();
	else if(storeGoals.poses[currentGoal-1].task == "bye")
		bye();
	else if(storeGoals.poses[currentGoal-1].task == "sleep")
		sleep();
}

void reset_pause(int* arr){
	for(int i = 0; i <= (pause_num-1); i++)
		arr[i] = 0;
}

int apply_pause(void){
	for(int i = 0; i <= (pause_num-1); i++){
		if(waypoint_num[i] == currentGoal)
			return pause_time[i];
	}
	return 0;
}

bool cancel_pause_handle(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res){
	reset_pause(waypoint_num);
	reset_pause(pause_time);
	pause_num = 0;
	return true;
}

bool pause_handle(waypoint_msgs::WaypointPauseTiming::Request &req, waypoint_msgs::WaypointPauseTiming::Response &res){
	if(pause_num < 10){
		waypoint_num[pause_num] = req.waypoint;
		pause_time[pause_num] = req.time;
		res.status = "[SUCCESS]: Timing applied";
		pause_num++;
	}
	else{
		res.status = "[ERROR]: Waypoint pause feature has been maxed out";
	}
	return true;
}

bool loop_handle(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
	if(req.data){
		loop_enabled = true;
		res.message = "Loop enabled";
	}
	else{
		loop_enabled = false;
		res.message = "Loop disabled";	
	}
	res.success = true;
	return true;
}

void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResult::ConstPtr& result)
{
	ROS_INFO("Waypoint %d has been completed.", currentGoal);
}

void callback_goal_reset(const move_base_msgs::MoveBaseActionResult::ConstPtr msg, MoveBaseClient* ac){
	if(msg->status.status == 3){
		task();
		if(currentGoal == amtOfGoals && loop_enabled){
			ros::Duration(apply_pause()).sleep();
			ROS_INFO("Looping waypoints");
			ROS_INFO("Waypoint 1 has been executed.");
			currentGoal = 1;
			move_base_msgs::MoveBaseGoal goal_msg;
			goal_msg.target_pose.pose = storeGoals.poses[currentGoal-1].pose;
			goal_msg.target_pose.header.frame_id = "map";
			ac->sendGoal(goal_msg, &doneCb);
		}
		else if(currentGoal<amtOfGoals)
		{
			ros::Duration(apply_pause()).sleep();
			move_base_msgs::MoveBaseGoal goal_msg;
			goal_msg.target_pose.pose = storeGoals.poses[currentGoal].pose;
			goal_msg.target_pose.header.frame_id = "map";
			ac->sendGoal(goal_msg, &doneCb);
			ROS_INFO("Waypoint %d has been executed.", (currentGoal+1));
			currentGoal++;
		}
		else{
			ROS_INFO("All waypoints have been completed!");
		}
	} 
}

void callback_data_path(const waypoint_msgs::PathTaskArray::ConstPtr& msg, MoveBaseClient* ac){
	currentGoal = 0;
	move_base_msgs::MoveBaseGoal goal_msg;
	storeGoals = *msg;
	amtOfGoals = msg->poses.size();
	goal_msg.target_pose.pose = storeGoals.poses[currentGoal].pose;
	goal_msg.target_pose.header.frame_id = "map";
	ac->sendGoal(goal_msg, &doneCb);
	currentGoal++;
	ROS_INFO("Waypoint %d has been executed with a total of %d waypoints.", currentGoal, amtOfGoals);
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "path_to_goal_waypoint");
	ros::NodeHandle nh;
	MoveBaseClient ac("move_base", true);
	//ac.waitForServer();
	ros::Subscriber sub_waypoints = nh.subscribe<waypoint_msgs::PathTaskArray>("/waypoints", 1000, boost::bind(callback_data_path, _1, &ac));
	ros::Subscriber waypoints_complete = nh.subscribe<move_base_msgs::MoveBaseActionResult>("/move_base/result", 1000, boost::bind(callback_goal_reset, _1, &ac));
	ros::ServiceServer server_pause = nh.advertiseService("/pause_waypoint", pause_handle);
	ros::ServiceServer server_pause_cancel = nh.advertiseService("/cancel_pause", cancel_pause_handle);
	ros::ServiceServer server_loop = nh.advertiseService("/loop_waypoint", loop_handle);

	ros::spin();
}
