#ifndef IDMIND_IMU_H
#define IDMIND_IMU_H

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include "idmind_serial/idmind_serial.h"

class IdmindIMU
{
public:
  IdmindIMU();
//  ~IdmindIMU();

  bool initialize();
  void runPeriodically();
  bool shutdown();

  bool green_light_;

private:
  void readPublish();

  ros::NodeHandle n_;
  ros::Publisher imu_pub_;
  ros::Timer timer_;

  std_msgs::Float64MultiArray inertial_;

  IdmindSerial serial_;

  std::string mode_;

  const double PI_;

  double update_frequency_;
};

#endif
