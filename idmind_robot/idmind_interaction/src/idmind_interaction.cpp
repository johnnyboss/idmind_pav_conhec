#include "idmind_interaction/idmind_interaction.h"

IdmindInteraction::IdmindInteraction()
    : n_("~"), serial_("/dev/idmind-interactionboard", 115200, 5), idmind_ns_("idmind_robot/"),
      capacitive_count_(5, 0), lights_(0, std::vector<uint8_t>(0)), last_head_vel_(0),
      green_light_(false), send_mouth_(false), send_lights_(false), send_head_(false)
{
  mouth_sub_ = n_.subscribe<std_msgs::UInt8MultiArray>("mouth", 10, &IdmindInteraction::mouthCallback, this);
  lights_sub_ = n_.subscribe<std_msgs::UInt8MultiArray>("lights", 10, &IdmindInteraction::lightsCallback, this);
  head_sub_ = n_.subscribe<std_msgs::UInt8MultiArray>("head", 10, &IdmindInteraction::headCallback, this);

  head_angle_pub_ = n_.advertise<std_msgs::UInt8>("head_angle", 100);
  capacitive_pub_ = n_.advertise<std_msgs::UInt8MultiArray>("capacitive", 100);

  projector_serv_ = n_.advertiseService("projector", &IdmindInteraction::projectorService, this);

  capacitive_calibration_ = n_.createTimer(ros::Duration(1800), &IdmindInteraction::calibrateCapacitiveTimer, this);

  ros::param::param<int>(idmind_ns_+"mouth_command", mouth_command_, 0x40);
  ros::param::param<int>(idmind_ns_+"lights_command", lights_command_, 0x43);
  ros::param::param<int>(idmind_ns_+"head_command", head_command_, 0x44);
  ros::param::param<int>(idmind_ns_+"projector_command", projector_command_, 0x47);
  ros::param::param<int>(idmind_ns_+"head_angle_command", head_angle_command_, 0x50);
  ros::param::param<int>(idmind_ns_+"capacitive_command", capacitive_command_, 0x52);

  ros::param::param<int>(idmind_ns_+"capacitive_true_positives", capacitive_true_positives_, 5);

  ros::param::param<bool>("~initialize", initialize_, false);

  mouth_[0] = mouth_command_;
  light_[0] = lights_command_;

  if (serial_.ready_)
  {
    ROS_INFO("\e[32m%s ---> SUCCESSFULL <---\e[0m", ros::this_node::getName().c_str());
    green_light_ = true;
  }
  else
    ROS_ERROR("%s ---> FAILED <---", ros::this_node::getName().c_str());
}

void IdmindInteraction::initialize()
{
  if (initialize_)
  {
    head_[0] = 70;
    head_[1] = 30;
    sendHeadCommand();
    ros::Duration(1.0).sleep();
    head_[0] = 110;
    sendHeadCommand();
    ros::Duration(2.0).sleep();
    head_[0] = 90;
    sendHeadCommand();
    ros::Duration(1.0).sleep();

    std::fill(mouth_+1, mouth_+sizeof(mouth_) ,255);
    sendMouthCommand();
    ros::Duration(1.0).sleep();
    std::fill(mouth_+1, mouth_+sizeof(mouth_) ,0);
    sendMouthCommand();
    ros::Duration(1.0).sleep();


    for (int i=0; i<6; i++)
    {
      std::vector<uint8_t> row(5, 0);
      row[0] = i;
      row[4] = 100;

      for (int j=1; j<4; j++)
      {
        row[j] = 100;
        lights_.push_back(row);
        sendLightsCommand();
        ros::Duration(1.0).sleep();
        row[j] = 0;
        lights_.push_back(row);
        sendLightsCommand();
        ros::Duration(1.0).sleep();
      }
    }

    lights_.clear();
  }
}

void IdmindInteraction::runPeriodically()
{
  calibrateCapacitive();

  ros::Rate r(10.0);
  while(n_.ok())
  {
    ros::spinOnce();

    if (send_mouth_) sendMouthCommand();
    ros::Duration(0.005).sleep();

    if (send_head_) sendHeadCommand();
    ros::Duration(0.005).sleep();

    if (send_lights_)
    {
      while(lights_.size() > 0)
      {
        sendLightsCommand();
        ros::Duration(0.005).sleep();
      }

      lights_.clear();
    }

    readPublishHeadAngle();
    ros::Duration(0.005).sleep();

    readPublishCapacitive();

    r.sleep();
  }
}

void IdmindInteraction::shutdown()
{
  ros::Duration(0.005).sleep();

  std::fill(mouth_+1, mouth_+sizeof(mouth_) , 0);
  sendMouthCommand();
  ros::Duration(0.005).sleep();

  lights_.clear();
  for (int i=0; i<6; i++)
  {
    std::vector<uint8_t> row(5, 0);
    row[0] = i;
    lights_.push_back(row);
    sendLightsCommand();
    ros::Duration(0.005).sleep();
  }

  head_[0] = 90;
  head_[1] = 50;
  sendHeadCommand();
}

void IdmindInteraction::mouthCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
  for (int i=0; i<32; i++)
    mouth_[i+1] = (*msg).data[i];

  send_mouth_ = true;
}

void IdmindInteraction::sendMouthCommand()
{
  if (serial_.write(mouth_, 33))
  {
    uint8_t buffer[4];
    if (!serial_.read(buffer, 4, true) || buffer[0] != mouth_command_)
      ROS_ERROR("%s --> Failed to send mouth command.", ros::this_node::getName().c_str());
  }

  send_mouth_ = false;
}

void IdmindInteraction::lightsCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
  std::vector<uint8_t> row;
  lights_.push_back(row);

  for (int i=0; i<5; i++)
    lights_[lights_.size()-1].push_back((*msg).data[i]);

  send_lights_ = true;
}

void IdmindInteraction::sendLightsCommand()
{
  for (int i=0; i<5; i++)
    light_[i+1] = lights_[lights_.size()-1][i];

  lights_.erase(lights_.end()-1);

  if (serial_.write(light_, 6))
  {
    uint8_t buffer[4];
    if (!serial_.read(buffer, 4, true) || buffer[0] != lights_command_)
      ROS_ERROR("%s --> Failed to send lights command (device %d).", ros::this_node::getName().c_str(), light_[1]);
  }

  send_lights_ = false;
}

void IdmindInteraction::headCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
  head_[0] = (*msg).data[0];
  head_[1] = (*msg).data[1];

  send_head_ = true;
}

void IdmindInteraction::sendHeadCommand()
{
  if (head_[1] != last_head_vel_)
  {
    uint8_t velocity[] = {head_command_+1, head_[1]};
    if (serial_.write(velocity, 2))
    {
      uint8_t buffer[4];
      if (!serial_.read(buffer, 4, true) || buffer[0] != head_command_+1)
        ROS_ERROR("%s --> Failed to send head velocity command.", ros::this_node::getName().c_str());
    }

    last_head_vel_ = head_[1];

    ros::Duration(0.005).sleep();
  }

  uint8_t position[] = {head_command_, head_[0]};
  if (serial_.write(position, 2))
  {
    uint8_t buffer[4];
    if (!serial_.read(buffer, 4, true) || buffer[0] != head_command_)
      ROS_ERROR("%s --> Failed to send head position command.", ros::this_node::getName().c_str());
  }

  send_head_ = false;
}

bool IdmindInteraction::projectorService(idmind_interaction::Projector::Request& req, idmind_interaction::Projector::Response& res)
{
  uint8_t command[] = {projector_command_, req.on};
  if (serial_.write(command, 2))
  {
    uint8_t buffer[4];
    if (serial_.read(buffer, 4, true) && buffer[0] == projector_command_)
    {
      res.success = true;
      return true;
    }
    else
      ROS_ERROR("%s --> Failed to send projector command (%d).", ros::this_node::getName().c_str(), req.on);
  }

  return false;
}

void IdmindInteraction::readPublishHeadAngle()
{
  if (serial_.write((uint8_t)head_angle_command_))
  {
    uint8_t buffer[5];
    if (serial_.read(buffer, 5, true) && buffer[0] == head_angle_command_)
    {
      std_msgs::UInt8 head_angle;
      head_angle.data = buffer[1];

//      ROS_INFO("%d | %d | %d | %d | %d", capacitive.data[0], capacitive.data[1],
//          capacitive.data[2], capacitive.data[3], capacitive.data[4]);

      head_angle_pub_.publish(head_angle);
    }
    else
      ROS_ERROR("%s --> Failed to read the head angle.", ros::this_node::getName().c_str());
  }
}

void IdmindInteraction::readPublishCapacitive()
{
  if (ros::Time::now() > capacitive_calibrated_ + ros::Duration(8.0))
  {
    if (serial_.write((uint8_t)capacitive_command_))
    {
      uint8_t buffer[6];
      if (serial_.read(buffer, 6, true) && buffer[0] == capacitive_command_)
      {
        std_msgs::UInt8MultiArray capacitive;
        capacitive.data.resize(5, 0);

        for (int i = 0; i < 5; i++)
        {
          int c = ((buffer[1] & static_cast<uint8_t>(pow(2, i))) != 0) ? 1 : 0;

          if (c == 1)
            ++capacitive_count_[i];
          else
            capacitive_count_[i] = 0;

          if (capacitive_count_[i] > capacitive_true_positives_)
          {
            capacitive.data[i] = 1;
            capacitive_count_[i] = 0;
          }

  //        ROS_INFO("%d | %d | %d | %d | %d", capacitive.data[0], capacitive.data[1],
  //            capacitive.data[2], capacitive.data[3], capacitive.data[4]);
        }

        capacitive_pub_.publish(capacitive);
      }
      else
        ROS_ERROR("%s --> Failed to read capacitive sensors.", ros::this_node::getName().c_str());
    }
  }
}

void IdmindInteraction::calibrateCapacitive()
{
  if (serial_.write((uint8_t)0x48))
  {
    uint8_t buffer[4];
    if (serial_.read(buffer, 4, true) && buffer[0] == 0x48)
    {
      ROS_INFO("%s --> Capacitive sensors calibrated.", ros::this_node::getName().c_str());

      capacitive_calibrated_ = ros::Time::now();
    }
    else
      ROS_ERROR("%s --> Failed to calibrate capacitive sensors.", ros::this_node::getName().c_str());
  }
}

void IdmindInteraction::calibrateCapacitiveTimer(const ros::TimerEvent&)
{
  calibrateCapacitive();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "idmind_interaction");

  IdmindInteraction interaction;

  interaction.initialize();

  if (interaction.green_light_)
    interaction.runPeriodically();

  interaction.shutdown();

  return 0;
}
