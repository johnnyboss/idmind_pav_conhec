#ifndef IDMIND_ODOMETRY_H
#define IDMIND_ODOMETRY_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>

struct RobotState
{
  double x;
  double y;
  double th;
  double vx;
  double vy;
  double vth;

  RobotState()  : x(0.0), y(0.0), th(0.0), vx(0.0), vy(0.0), vth(0.0)
  {}
};

class IdmindOdometry
{
public:
  IdmindOdometry();
//  ~IdmindOdometry();

  void runPeriodically();

private:
  void encodersCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);
  void inertialCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

  void calculateState();
  void publishState();

  void differential(double* delta_x, double* delta_y, double* delta_angular);
  void holonomic(double* delta_x, double* delta_y, double* delta_angular);

  ros::NodeHandle n_;
  ros::Subscriber encoders_sub_;
  ros::Subscriber inertial_sub_;
  ros::Publisher odom_pub_;
  tf::TransformBroadcaster odom_broadcaster_;

  std_msgs::Int32MultiArray encoders_;
  std_msgs::Float64MultiArray inertial_;

  ros::Time last_time_enc_, last_time_imu_;

  RobotState state_;

  std::string idmind_ns_, kinematics_;

  const double PI_;

  double wheel_radius_;
  double base_width_;

  double dt_odo_, dt_imu_;
  double imu_offset_;

  int wheels_;
  int ticks_;

  bool new_encoder_, new_imu_;
  bool use_imu_;
  bool publish_;
};

#endif
