#ifndef IDMIND_TELEOPERATION_H
#define IDMIND_TELEOPERATION_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include "idmind_motors/Hardstop.h"

class IdmindTeleoperation
{
public:
  IdmindTeleoperation();
//  ~IdmindTeleoperation();

  void runPeriodically();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
  void serviceCheck(const ros::TimerEvent&);

  ros::NodeHandle n_;
  ros::Subscriber joy_sub_;
  ros::Publisher twist_pub_;
  ros::Publisher chatter_pub_;

  ros::ServiceClient hardstop_client_;

  ros::Timer service_check_;
  ros::Time last_received_, last_button_;

  geometry_msgs::Twist twist_;

  int latched_counter_;
  int zeroes_;

  bool autonomous_, latched_, publishing_, reconnecting_;
};

#endif
