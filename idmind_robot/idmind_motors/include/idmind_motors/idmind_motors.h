#ifndef IDMIND_MOTORS_H
#define IDMIND_MOTORS_H

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include "idmind_serial/idmind_serial.h"
#include "idmind_motors/Hardstop.h"
#include "idmind_motors/VoltagesStatus.h"

class IdmindMotors
{
public:
  IdmindMotors();
//  ~IdmindMotors();

  void runPeriodically();

  bool green_light_;

private:
  void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
  bool hardstopService(idmind_motors::Hardstop::Request& req, idmind_motors::Hardstop::Response& res);
  bool hardstopTimer(int duration);
  void hardstopReset(const ros::TimerEvent&);
  bool voltagesService(idmind_motors::VoltagesStatus::Request& req, idmind_motors::VoltagesStatus::Response& res);

  void readPublishEncoders();
  void sendVelocityCommand();
  void inverseKinematics(std::vector<int>* angular_vels);
  void velocitySmoother();

  int sign(double number);

  ros::NodeHandle n_;
  ros::Subscriber twist_sub_;
  ros::Publisher encoders_pub_;
  ros::ServiceServer hardstop_serv_, voltages_status_serv_;

  ros::Timer hardstop_timer_;
  ros::Time last_received_;

  std_msgs::Int32MultiArray encoders_;
  geometry_msgs::Twist twist_, last_twist_;

  IdmindSerial serial_;

  std::string idmind_ns_, robot_, kinematics_;

  double frequency_;

  double wheel_radius_;
  double base_width_;
  double watchdog_time_;
  double max_acc_v_, max_dec_v_, max_acc_w_, max_dec_w_, acc_dec_factor_;
  double max_v_, max_w_;

  int wheels_;
  int velocity_offset_;
  int encoders_command_;
  int velocity_command_;
  int voltages_command_;
  int stop_command_;

  int hardstop_blocked_;

  bool once_;
};

#endif
