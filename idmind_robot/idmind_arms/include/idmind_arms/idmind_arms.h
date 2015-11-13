#ifndef IDMIND_ARMS_H
#define IDMIND_ARMS_H

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16MultiArray.h>
#include "idmind_serial/idmind_serial.h"
#include "idmind_arms/Torque.h"

class IdmindArms
{
public:
  IdmindArms();
//  ~IdmindArms();

  void initialize();
  void runPeriodically();
  void shutdown();

  bool green_light_;

private:
  void positionCallback(const std_msgs::UInt16MultiArray::ConstPtr& msg);
  void sendPositionCommand();

  bool torqueService(idmind_arms::Torque::Request& req, idmind_arms::Torque::Response& res);
  bool sendTorqueCommand(int mode_left, int mode_right);

  int readRAM(int ID, int memory, int number_bytes);
  void readTorque();

  ros::NodeHandle n_;
  ros::Subscriber position_sub_;
//  ros::Publisher status_pub_;

  ros::ServiceServer torque_serv_;

  IdmindSerial serial_;

  std::vector<int> position_, play_time_, last_position_, send_position_;
  std::vector<int> mode_;

  double time_constant_;

  int min_ticks_, max_ticks_;
  int range_;

  int mode_ON_;
};

#endif
