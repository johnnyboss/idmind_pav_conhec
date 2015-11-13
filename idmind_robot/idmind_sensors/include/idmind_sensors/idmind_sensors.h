#ifndef IDMIND_SENSORS_H
#define IDMIND_SENSORS_H

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include "idmind_serial/idmind_serial.h"
#include "idmind_motors/Hardstop.h"
#include "idmind_sensors/Batteries.h"
#include "idmind_sensors/DockUndock.h"

class IdmindSensors
{
public:
  IdmindSensors();
//  ~IdmindSensors();

  void runPeriodically();

  bool green_light_;

private:
  bool batteriesService(idmind_sensors::Batteries::Request& req, idmind_sensors::Batteries::Response& res);
  bool readBatteries(std::vector<double>* voltages);

  bool dockUndockService(idmind_sensors::DockUndock::Request& req, idmind_sensors::DockUndock::Response& res);
  int sendCommandControl(int command, int control);

  void sendHardstop();
  void serviceCheck(const ros::TimerEvent&);

  void readPublishGroundSensors();

  ros::NodeHandle n_;
  ros::Publisher ground_sensors_pub_, twist_pub_;
  ros::ServiceServer batteries_serv_, dock_undock_serv_;

  ros::ServiceClient hardstop_client_;

  ros::Timer batteries_check_, service_check_;
  ros::Time last_hardstop_;

  IdmindSerial serial_;

  std::string idmind_ns_;

  std::vector<int> ground_min_;

  double frequency_;

  int ground_sensors_command_;
  int batteries_command_;

  bool reconnecting_;
};

#endif
