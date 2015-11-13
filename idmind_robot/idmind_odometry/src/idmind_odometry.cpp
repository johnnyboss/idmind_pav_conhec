#include "idmind_odometry/idmind_odometry.h"

IdmindOdometry::IdmindOdometry()
    : n_("~"), idmind_ns_("idmind_robot/"), PI_(3.1415926536), dt_odo_(0.0), dt_imu_(0.0), imu_offset_(0.0),
      new_encoder_(false), new_imu_(false), publish_(false)
{
  ros::param::param<bool>("~use_imu", use_imu_, true);

  encoders_sub_ = n_.subscribe<std_msgs::Int32MultiArray>("/idmind_motors/encoders", 100, &IdmindOdometry::encodersCallback, this);
  if (use_imu_)
    inertial_sub_ = n_.subscribe<std_msgs::Float64MultiArray>("/idmind_imu/inertial", 100, &IdmindOdometry::inertialCallback, this);
  odom_pub_ = n_.advertise<nav_msgs::Odometry>("/odom", 100);

  ros::param::get(idmind_ns_+"kinematics", kinematics_);
  ros::param::get(idmind_ns_+"wheels", wheels_);
  ros::param::get(idmind_ns_+"wheel_radius", wheel_radius_);
  ros::param::get(idmind_ns_+"base_width", base_width_);
  ros::param::get(idmind_ns_+"ticks", ticks_);

  encoders_.data.resize(wheels_, 0);
  inertial_.data.resize(3, 0.0);

  last_time_enc_ = ros::Time::now();
  last_time_imu_ = ros::Time::now();

  ROS_INFO("\e[32m%s ---> SUCCESSFULL <---\e[0m", ros::this_node::getName().c_str());
}

void IdmindOdometry::runPeriodically()
{
  ros::Rate r(80.0);
  while(n_.ok())
  {
    ros::spinOnce();
    calculateState();
    publishState();

    r.sleep();
  }
}

void IdmindOdometry::encodersCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
  if (new_encoder_)
    ROS_WARN("Encoder doubled!");

  ros::Time current_time = ros::Time::now();
  dt_odo_ = (current_time - last_time_enc_).toSec();
  last_time_enc_ = current_time;
  encoders_ = *msg;
  new_encoder_ = true;
}

void IdmindOdometry::inertialCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  if (new_imu_)
    ROS_WARN("IMU doubled!");

  ros::Time current_time = ros::Time::now();
  dt_imu_ = (current_time - last_time_imu_).toSec();
  last_time_imu_ = current_time;
  inertial_ = *msg;
  new_imu_ = true;
}

void IdmindOdometry::calculateState()
{
  if (new_encoder_ && (new_imu_ || !use_imu_))
  {
//    ROS_INFO("time: %.2f", (last_time_enc_ - last_time_imu_).toNSec()*1e-6);

    double delta_x, delta_y, delta_angular = 0.0;

    if (use_imu_)
    {
      if (imu_offset_ == 0.0)
        imu_offset_ = inertial_.data[0];

      delta_angular = (inertial_.data[0] - imu_offset_) - state_.th;

      if (delta_angular > PI_)
        delta_angular -= 2*PI_;
      else if (delta_angular < -PI_)
        delta_angular += 2*PI_;

      state_.vth = delta_angular / dt_imu_;
    }

    if (kinematics_ == "differential")
      differential(&delta_x, &delta_y, &delta_angular);
    else if (kinematics_ == "omnidirectional")
      holonomic(&delta_x, &delta_y, &delta_angular);

    state_.x += delta_x;
    state_.y += delta_y;
    state_.th += delta_angular;

    new_encoder_ = new_imu_ = false;
    publish_ = true;
  }
}

void IdmindOdometry::publishState()
{
  if (publish_)
  {
    ros::Time current_time = ros::Time::now();

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(state_.th);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = state_.x;
    odom_trans.transform.translation.y = state_.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster_.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = state_.x;
    odom.pose.pose.position.y = state_.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.twist.twist.linear.x = state_.vx;
    odom.twist.twist.linear.y = state_.vy;
    odom.twist.twist.angular.z = state_.vth;

    odom_pub_.publish(odom);

    publish_ = false;
  }
}

void IdmindOdometry::differential(double* delta_x, double* delta_y, double* delta_angular)
{
  //double& delta_ang = *delta_angular; //Use reference instead of dereference.

  double dl = encoders_.data[0] * 2 * PI_ * wheel_radius_ / ticks_;
  double dr = encoders_.data[1] * 2 * PI_ * wheel_radius_ / ticks_;

  double delta_linear = (dl + dr) / 2;
  state_.vx = delta_linear / dt_odo_;

  if (!use_imu_)
  {
    *delta_angular = (-dl + dr) / base_width_;
    state_.vth = *delta_angular / dt_odo_;
  }

  if (fabs(*delta_angular) < 1e-2)
  {
    *delta_x = delta_linear * cos(state_.th + *delta_angular / 2);
    *delta_y = delta_linear * sin(state_.th + *delta_angular / 2);
  }
  else
  {
    *delta_x = (delta_linear / *delta_angular) * (sin(state_.th + *delta_angular) - sin(state_.th));
    *delta_y = -(delta_linear / *delta_angular) * (cos(state_.th + *delta_angular) - cos(state_.th));
  }
}

void IdmindOdometry::holonomic(double* delta_x, double* delta_y, double* delta_angular)
{
  std::vector<double> d_wheels(4, 0.0);

  for (int i = 0; i < 4; i++)
    d_wheels[i] = encoders_.data[i] * 2 * PI_ / ticks_;

  double delta_linear_x = (wheel_radius_ / 4) * (d_wheels[0] + d_wheels[1] + d_wheels[2] + d_wheels[3]);
  double delta_linear_y = (wheel_radius_ / 4) * (-d_wheels[0] + d_wheels[1] + d_wheels[2] - d_wheels[3]);
  state_.vx = delta_linear_x / dt_odo_;
  state_.vy = delta_linear_y / dt_odo_;

  if (!use_imu_)
  {
    *delta_angular = (wheel_radius_ / (4*base_width_)) * (d_wheels[0] - d_wheels[1] + d_wheels[2] - d_wheels[3]);
    state_.vth = *delta_angular / dt_odo_;
  }

  *delta_x = delta_linear_x * cos(state_.th + *delta_angular) - delta_linear_y * sin(state_.th + *delta_angular);
  *delta_y = delta_linear_x * sin(state_.th + *delta_angular) + delta_linear_y * cos(state_.th + *delta_angular);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "idmind_odometry");

  IdmindOdometry odometry;
  odometry.runPeriodically();

  return 0;
}
