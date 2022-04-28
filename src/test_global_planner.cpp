#include <iostream>
#include <thread>
#include <vector>

#include <std_srvs/Empty.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>

#include "GlobalParams.h"
#include "GlobalPlanner.h"

#include <ompl/util/RandomNumbers.h>
#include <ompl/base/goals/GoalSpace.h>
#include <ompl/control/SimpleDirectedControlSampler.h>


unsigned got_init_pose = 0;
geometry_msgs::Pose initial_pose;

void get_pos_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  initial_pose = msg->pose.pose;
  got_init_pose = 1;
}


//no move base. just call global planner
int main(int argc, char **argv){
  ROS_INFO("Starting up auvsl_rrt_node\n");
  ros::init(argc, argv, "auvsl_rrt_planner");
  ros::NodeHandle nh;
  GlobalParams::load_params(&nh);
  ros::Rate loop_rate(10);

  std::vector<geometry_msgs::PoseStamped> plan;
  geometry_msgs::PoseStamped startp;
  geometry_msgs::PoseStamped goalp;

  startp.pose.position.x = 0;
  startp.pose.position.y = 0;
  startp.pose.position.z = .16;

  startp.pose.orientation.x = 0;
  startp.pose.orientation.y = 0;
  startp.pose.orientation.z = 0;
  startp.pose.orientation.w = 1;


  goalp.pose.position.x = 0;
  goalp.pose.position.y = 10;
  goalp.pose.position.z = .16;

  goalp.pose.orientation.x = 0;
  goalp.pose.orientation.y = 0;
  goalp.pose.orientation.z = 0;
  goalp.pose.orientation.w = 1;

  
  auvsl::GlobalPlanner planner;
  planner.initialize("Global Planner", NULL);
  planner.makePlan(startp, goalp, plan);

  return 1;
}
