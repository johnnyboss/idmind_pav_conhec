#include "idmind_motors/idmind_motors.h"

IdmindMotors::IdmindMotors()
    : n_("~"), serial_("/dev/idmind-motorsboard", 115200, 5), idmind_ns_("idmind_robot/"), frequency_(0.0),
      hardstop_blocked_(0), once_(true), green_light_(false)
{
  twist_sub_ = n_.subscribe<geometry_msgs::Twist>("twist", 1, &IdmindMotors::twistCallback, this);
  encoders_pub_ = n_.advertise<std_msgs::Int32MultiArray>("encoders", 100);

  hardstop_serv_ = n_.advertiseService("hardstop", &IdmindMotors::hardstopService, this);
  voltages_status_serv_ = n_.advertiseService("voltages_status", &IdmindMotors::voltagesService, this);

  hardstop_timer_ = n_.createTimer(ros::Duration(0.0), &IdmindMotors::hardstopReset, this, true, false);

  ros::param::param<std::string>(idmind_ns_+"robot", robot_, "generic");
  ros::param::get(idmind_ns_+"kinematics", kinematics_);
  ros::param::get(idmind_ns_+"wheels", wheels_);
  ros::param::get(idmind_ns_+"wheel_radius", wheel_radius_);
  ros::param::get(idmind_ns_+"base_width", base_width_);
  ros::param::param<double>(idmind_ns_+"max_acc_v", max_acc_v_, 0.5);
  ros::param::param<double>(idmind_ns_+"max_acc_w", max_acc_w_, 2.0);
  ros::param::param<double>(idmind_ns_+"acc_del_factor", acc_dec_factor_, 5.0);
  ros::param::param<double>(idmind_ns_+"max_v", max_v_, 1.0);
  ros::param::param<double>(idmind_ns_+"max_w", max_w_, 2.0);
  ros::param::param<int>(idmind_ns_+"velocity_offset", velocity_offset_, 0);
  ros::param::param<int>(idmind_ns_+"encoders_command", encoders_command_, 0x4A);
  ros::param::param<int>(idmind_ns_+"velocity_command", velocity_command_, 0x56);
  ros::param::param<int>(idmind_ns_+"voltages_command", voltages_command_, 0x51);
  ros::param::param<int>(idmind_ns_+"stop_command", stop_command_, 0x57);

  ros::param::param<double>(idmind_ns_+"watchdog_time", watchdog_time_, 0.060);

  max_dec_v_ = acc_dec_factor_ * max_acc_v_;
  max_dec_w_ = acc_dec_factor_ * max_acc_w_;

  encoders_.data.resize(wheels_, 0);

  if (serial_.ready_)
  {
    ROS_INFO("\e[32m%s ---> SUCCESSFULL <---\e[0m", ros::this_node::getName().c_str());
    green_light_ = true;
  }
  else
    ROS_ERROR("%s ---> FAILED <---", ros::this_node::getName().c_str());
}

void IdmindMotors::runPeriodically()
{
  frequency_ = 20.0;
  ros::Rate r(frequency_);
  while(n_.ok())
  {
    ros::spinOnce();

    sendVelocityCommand();
    ros::Duration(0.010).sleep();
    readPublishEncoders();

    r.sleep();
  }
}

void IdmindMotors::twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  if (hardstop_blocked_ == 0)
  {
    last_received_ = ros::Time::now();
    last_twist_ = twist_;
    twist_ = *msg;
  }
}

bool IdmindMotors::hardstopService(idmind_motors::Hardstop::Request& req, idmind_motors::Hardstop::Response& res)
{
  if ((req.activate == 1 && req.hardstop_time == 0) || (req.activate == 0 && req.hardstop_time != 0))
    return false;

  if ((req.activate == 1 && hardstop_blocked_ == 0) || req.activate == 0)
  {
    ROS_WARN("%s --> Hardstop %d.", ros::this_node::getName().c_str(), req.activate);

    if (req.activate == 0)
      res.success = hardstopTimer(0);
    else if (req.hardstop_time == 2)
      res.success = true;
    else
      res.success = hardstopTimer(req.hardstop_time);

    if (serial_.write((uint8_t)stop_command_))
    {
      uint8_t buffer[4];
      if (!serial_.read(buffer, 4, true) || buffer[0] != stop_command_)
      {
        ROS_ERROR("%s --> Failed to set hardstop.", ros::this_node::getName().c_str());
        res.success = false;
      }

      ros::Duration(0.001).sleep();
    }
  }

  if (req.activate == 1)
  {
    ros::Duration duration = ros::Duration(req.hardstop_time + (req.back_up == 1 ? 1.0 : 0.0));
    hardstop_timer_.setPeriod(duration);
    hardstop_timer_.start();

    twist_.linear.x = req.back_up == 1 ? -0.6 : 0.0;
    twist_.linear.y = twist_.angular.z = 0.0;

    hardstop_blocked_ = req.stay_blocked == 1 ? 2 : 1;

    return res.success;
  }
  else if (req.activate == 0)
  {
    hardstopTimer(2);
    hardstop_blocked_ = 0;
    return res.success;
  }

  return false;
}

bool IdmindMotors::hardstopTimer(int duration)
{
  uint8_t command[2] = {stop_command_+1, duration};
  if (serial_.write(command, 2))
  {
    uint8_t buffer[4];
    if (serial_.read(buffer, 4, true) && buffer[0] == stop_command_+1)
    {
      ros::Duration(0.001).sleep();
      return true;
    }
  }

  ROS_ERROR("%s --> Failed to set hardstop timer.", ros::this_node::getName().c_str());
  return false;
}

void IdmindMotors::hardstopReset(const ros::TimerEvent&)
{
  hardstopTimer(2);

  twist_.linear.x = 0.0;

  if (hardstop_blocked_ == 1)
    hardstop_blocked_ = 0;
}

bool IdmindMotors::voltagesService(idmind_motors::VoltagesStatus::Request& req, idmind_motors::VoltagesStatus::Response& res)
{
  if (serial_.write((uint8_t)voltages_command_))
  {
    uint8_t buffer[8];
    if (serial_.read(buffer, 8, true) && buffer[0] == voltages_command_)
    {
      std::vector<double> voltages(3, 0.0);

      for (int i = 0; i < 3; i++)
        voltages[i] = static_cast<double>(buffer[i+1]) / 10.0;

      res.motors_voltage = voltages[0];
      res.drivers_voltage = voltages[1];
      res.electronics_voltage = voltages[2];
      res.motor_drivers = buffer[4] & 0x80 != 0 ? 1 : 0;
      res.motors_power = buffer[4] & 0x04 != 0 ? 1 : 0;
      res.drivers_power = buffer[4] & 0x02 != 0 ? 1 : 0;
      res.electronics_power = buffer[4] & 0x01 != 0 ? 1 : 0;

      res.success = true;
      return true;
    }
    else
    {
      ROS_ERROR("%s --> Failed to read voltages.", ros::this_node::getName().c_str());
    }
  }

  return false;
}

void IdmindMotors::readPublishEncoders()
{
  int size = 4 + 2*wheels_;

  if (serial_.write((uint8_t)encoders_command_))
  {
    uint8_t buffer[size];
    if (serial_.read(buffer, size, true) && buffer[0] == encoders_command_)
    {
      encoders_.data[0] = -serial_.readInt(&buffer[1]);
      encoders_.data[1] = serial_.readInt(&buffer[3]);

      if (wheels_ == 4)
      {
        encoders_.data[2] = -serial_.readInt(&buffer[5]);
        encoders_.data[3] = serial_.readInt(&buffer[7]);
      }

      if (once_)
      {
        encoders_.data.resize(wheels_, 0);
        ROS_INFO("%s --> Initialized encoders.", ros::this_node::getName().c_str());
        once_ = false;
      }

//      ROS_INFO("%d | %d | %d | %d", encoders_.data[0], encoders_.data[1], encoders_.data[2], encoders_.data[3]);

      encoders_pub_.publish(encoders_);
    }
    else
      ROS_ERROR("%s --> Failed to read encoders.", ros::this_node::getName().c_str());
  }
}

void IdmindMotors::sendVelocityCommand()
{
  std::vector<int> angular_vels(wheels_, 0);

  if (kinematics_.find("calibration") == -1)
  {
    if (ros::Time::now() > (last_received_ + ros::Duration(watchdog_time_)) && hardstop_blocked_ == 0)
    {
      if (twist_.linear.x != 0.0 || twist_.linear.y != 0.0 || twist_.angular.z != 0.0)
        ROS_WARN("%s --> WATCHDOG!", ros::this_node::getName().c_str());

      twist_.linear.x = twist_.linear.y = twist_.angular.z = 0.0;
    }
    else
    {
      twist_.linear.x  =
          twist_.linear.x  > 0.0 ? std::min(twist_.linear.x,  max_v_) : std::max(twist_.linear.x,  -max_v_);
      twist_.linear.y  =
          twist_.linear.y  > 0.0 ? std::min(twist_.linear.y,  max_v_) : std::max(twist_.linear.y,  -max_v_);
      twist_.angular.z =
          twist_.angular.z > 0.0 ? std::min(twist_.angular.z, max_w_) : std::max(twist_.angular.z, -max_w_);

      velocitySmoother();

      if (hardstop_blocked_ != 0 && twist_.linear.x != 0.0)
        twist_.linear.x = -0.5;
    }
  }

  inverseKinematics(&angular_vels);

  int size = 1 + 2*wheels_;
  uint8_t command[size];
  command[0] = velocity_command_;
  serial_.addInt(&command[1], angular_vels[0]);
  serial_.addInt(&command[3], angular_vels[1]);

  if (wheels_ == 4)
  {
    serial_.addInt(&command[5], angular_vels[2]);
    serial_.addInt(&command[7], angular_vels[3]);
  }

  if (serial_.write(command, size))
  {
    uint8_t buffer[4];
    if (!serial_.read(buffer, 4, true) || buffer[0] != velocity_command_)
      ROS_ERROR("%s --> Failed to send velocity command.", ros::this_node::getName().c_str());
//    else
//      ROS_INFO("--> vel Left: %d | Right: %d", left_wheel_vel, right_wheel_vel);
  }
}

int IdmindMotors::sign(double number)
{
  if (number > 1e-6)
    return 1;
  else if (number < -1e-6)
    return -1;
  else
    return 0;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "idmind_motors");

  IdmindMotors motors;

  if (motors.green_light_)
    motors.runPeriodically();

  return 0;
}
