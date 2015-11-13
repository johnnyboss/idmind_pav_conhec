#ifndef IDMIND_BEHAVIOURS_H
#define IDMIND_BEHAVIOURS_H

#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>
#include <tf/transform_datatypes.h>

struct Colour
{
  int red;
  int green;
  int blue;
};

struct Behaviour
{
  int head;
  int left_arm;
  int right_arm;
  int mouth;
  std::vector<int> leds;
};

struct Media
{
  std::string type;
  std::string file_name;
};

struct Point
{
  double x;
  double y;
  double th;

  int stop;
  int behaviour;
  int media;
};

class IdmindBehaviours
{
public:
  IdmindBehaviours();
//  ~IdmindBehaviours();

  void runPeriodically();

  bool green_light_;

private:
  void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

  void playMedia(int i);
  void sendBehaviour(int i);

  bool readFiles();

  template<typename T>
  bool readNumbersFile(std::string file, std::vector<std::vector<T> >* vector);
  bool readStringsFile(std::string file, std::vector<std::vector<std::string> >* vector);

  ros::NodeHandle n_;
  ros::Subscriber amcl_pose_sub_;
  ros::Publisher head_pub_, mouth_pub_, lights_pub_, arms_pub_;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client_;

  move_base_msgs::MoveBaseGoal goal_;

  ros::Time last_aborted_;

  std::vector<Colour> colours_;
  std::vector<Behaviour> behaviours_;
  std::vector<Media> media_;
  std::vector<Point> points_;

  std::vector<double> current_position_;

  std::string behaviour_ns_, behaviours_folder_;

  double goal_radius_;
  double wake_up_time_, nap_time_, nap_duration_, sleep_time_;

  int target_, tentatives_;

  bool to_next_target_, cancelled_;
};

#endif
