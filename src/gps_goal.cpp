#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_localization/navsat_conversions.h>
#include <geometry_msgs/PointStamped.h>
// #include <std_msgs/Bool.h>
// #include <tf/transform_listener.h>
#include <math.h>
#include <fstream>
#include <utility>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <math.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
// MoveBaseClient; // create a type definition for a client called MoveBaseClient

std::vector<std::pair<double, double>> waypointVect;
std::vector<std::pair<double, double>>::iterator iter; // init. iterator
geometry_msgs::PointStamped UTM_point, map_point, UTM_next, map_next;
int count = 0, waypointCount = 0, wait_count = 0;
double numWaypoints = 0;
double latiGoal, longiGoal, latiNext, longiNext;
std::string utm_zone;
std::string path_local, path_abs;

geometry_msgs::PointStamped latLongtoUTM(double lati_input, double longi_input)
{
  double utm_x = 0, utm_y = 0;
  geometry_msgs::PointStamped UTM_point_output;
  // convert lat/long to utm
  // utm_zone = "32"; 
  RobotLocalization::NavsatConversions::LLtoUTM(lati_input, longi_input, utm_y, utm_x, utm_zone);

  // Construct UTM_point and map_point geometry messages
  UTM_point_output.header.frame_id = "utm";
  UTM_point_output.header.stamp = ros::Time(0);
  UTM_point_output.point.x = utm_x;
  UTM_point_output.point.y = utm_y;
  UTM_point_output.point.z = 0;

  return UTM_point_output;
}
geometry_msgs::PointStamped UTMtoMapPoint(geometry_msgs::PointStamped UTM_input)
{
  geometry_msgs::PointStamped map_point_output;
  bool notDone = true;
  tf::TransformListener listener; // create transformlistener object called listener
  ros::Time time_now = ros::Time::now();
  while (notDone)
  {
    try
    {
      UTM_point.header.stamp = ros::Time::now();
      listener.waitForTransform("odom", "utm", time_now, ros::Duration(3.0));
      listener.transformPoint("odom", UTM_input, map_point_output);
      notDone = false;
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(0.01).sleep();
      // return;
    }
  }
  return map_point_output;
}
void send_goals(geometry_msgs::PointStamped map_point)
{
  MoveBaseClient ac("move_base", true);

  // wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "odom";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = map_point.point.x; // specify x goal
  goal.target_pose.pose.position.y = map_point.point.y; // specify y goal

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_navigation_goals");
  latiGoal = 48;
  longiGoal = 38;
  UTM_point = latLongtoUTM(latiGoal, longiGoal);
  map_point = UTMtoMapPoint(UTM_point);
  send_goals(map_point);
  // tell the action client that we want to spin a thread by default

  return 0;
}