#ifndef IDMIND_INTERACTION_H
#define IDMIND_INTERACTION_H

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include "idmind_serial/idmind_serial.h"
#include "idmind_interaction/Projector.h"

class IdmindInteraction
{
public:
  IdmindInteraction();
//  ~IdmindInteraction();

  void initialize();
  void runPeriodically();
  void shutdown();

  bool green_light_;

private:
  void mouthCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg);
  void sendMouthCommand();
  void lightsCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg);
  void sendLightsCommand();
  void headCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg);
  void sendHeadCommand();

  bool projectorService(idmind_interaction::Projector::Request& req, idmind_interaction::Projector::Response& res);

  void readPublishHeadAngle();
  void readPublishCapacitive();
  void calibrateCapacitive();
  void calibrateCapacitiveTimer(const ros::TimerEvent&);

  ros::NodeHandle n_;
  ros::Subscriber mouth_sub_, lights_sub_, head_sub_;
  ros::Publisher head_angle_pub_, capacitive_pub_;

  ros::ServiceServer projector_serv_;

  ros::Timer capacitive_calibration_;
  ros::Time capacitive_calibrated_;

  IdmindSerial serial_;

  std::string idmind_ns_;

  std::vector<int> capacitive_count_;
  int capacitive_true_positives_;

  int mouth_command_;
  int lights_command_;
  int head_command_;
  int projector_command_;
  int head_angle_command_;
  int capacitive_command_;

  uint8_t mouth_[33];
  uint8_t head_[2];

  uint8_t light_[6];
  std::vector<std::vector<uint8_t> > lights_;

  uint8_t last_head_vel_;

  bool send_mouth_, send_lights_, send_head_;
  bool initialize_;
};

#endif
