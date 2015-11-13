#include "idmind_imu/idmind_imu.h"

IdmindIMU::IdmindIMU()
    : n_("~"), serial_("/dev/idmind-imu", 115200, 5), PI_(3.1415926536), green_light_(false)
{
  imu_pub_ = n_.advertise<std_msgs::Float64MultiArray>("inertial", 100);
//  timer_ = n_.createTimer(ros::Duration(5), &IdmindSerial::checkPorts, &serial_);

  ros::param::param<std::string>("~mode", mode_, "sample");
  ros::param::param<double>("~update_frequency", update_frequency_, 20.0);

  inertial_.data.resize(3, 0.0);

//  ros::Duration(3).sleep();

  if (initialize())
  {
    ROS_INFO("\e[32m%s ---> SUCCESSFULL <---\e[0m", ros::this_node::getName().c_str());
    green_light_ = true;
  }
  else
    ROS_ERROR("%s ---> FAILED <---", ros::this_node::getName().c_str());
}

bool IdmindIMU::initialize()
{
  int mode;
  if (mode_ == "stream")
    mode = 0x01;
  else if (mode_ == "sample")
    mode = 0x02;

  uint8_t command[2] = {0x50, mode};

  if (serial_.write(command, 2))
  {
    uint8_t buffer[4];
    if (serial_.read(buffer, 4, true) && buffer[0] == 0x50)
    {
      serial_.flush();
      return true;
    }
  }

  return false;
}

void IdmindIMU::runPeriodically()
{
  double rate;
  if (mode_ == "stream")
    rate = 110.0;
  else if  (mode_ == "sample")
    rate = update_frequency_;

  ros::Rate r(rate);

  while(n_.ok())
  {
    ros::spinOnce();
    readPublish();

    r.sleep();
  }
}

bool IdmindIMU::shutdown()
{
  uint8_t command[2] = {0x50, 0x00};
  if (serial_.write(command, 2))
  {
    uint8_t buffer[4];
    if (serial_.read(buffer, 4, true) && buffer[0] == 0x50)
    {
      return true;
    }
  }

  return false;
}

void IdmindIMU::readPublish()
{
//  if (serial_.reconnected_)
//  {
//    initialize();
//    serial_.reconnected_ = false;
//  }

  if (serial_.ready_)
  {
    if (mode_ == "sample")
      if (!serial_.write((uint8_t)0x40))
        ROS_ERROR("%s --> Failed to ask IMU.", ros::this_node::getName().c_str());

    uint8_t buffer[10];
    if (serial_.read(buffer, 10, true) && buffer[0] == 0x40)
    {
      inertial_.data[0] = -((double)serial_.readInt(&buffer[1]) / 100.0) * PI_ / 180;
      inertial_.data[1] = ((double)serial_.readInt(&buffer[3]) / 100.0) * PI_ / 180;
      inertial_.data[2] = ((double)serial_.readInt(&buffer[5]) / 100.0) * PI_ / 180;

      imu_pub_.publish(inertial_);
    }
    else
      ROS_ERROR("%s --> Failed to read IMU.", ros::this_node::getName().c_str());
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "idmind_imu");

  IdmindIMU imu;

  if (imu.green_light_)
  {
    imu.runPeriodically();
    if (!imu.shutdown())
      printf("\e[31mFailed to shutdown IMU.\n\e[0m");
  }

  return 0;
}
