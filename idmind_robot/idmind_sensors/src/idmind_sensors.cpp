#include "idmind_sensors/idmind_sensors.h" // Add voltage check on docking!

IdmindSensors::IdmindSensors()
    : n_("~"), serial_("/dev/idmind-sensorsboard", 115200, 5), idmind_ns_("idmind_robot/"), ground_min_(4, 80),
      frequency_(0.0), green_light_(false), reconnecting_(false)
{
  ground_sensors_pub_ = n_.advertise<std_msgs::Int32MultiArray>("ground_sensors", 100);
  twist_pub_ = n_.advertise<geometry_msgs::Twist>("/idmind_motors/twist", 1);

  batteries_serv_ = n_.advertiseService("batteries", &IdmindSensors::batteriesService, this);
  dock_undock_serv_ = n_.advertiseService("dock_undock", &IdmindSensors::dockUndockService, this);

  hardstop_client_ = n_.serviceClient<idmind_motors::Hardstop>("/idmind_motors/hardstop", true);
  hardstop_client_.waitForExistence();

  service_check_ = n_.createTimer(ros::Duration(0.1), &IdmindSensors::serviceCheck, this);

  last_hardstop_ = ros::Time::now();

  ros::param::param<int>(idmind_ns_+"batteries_command", batteries_command_, 0x51);
  ros::param::param<int>(idmind_ns_+"ground_sensors_command", ground_sensors_command_, 0x59);

  ground_min_[0] = 20; ground_min_[1] = 30; ground_min_[2] = 30; ground_min_[3] = 30;

  if (serial_.ready_)
  {
    ROS_INFO("\e[32m%s ---> SUCCESSFULL <---\e[0m", ros::this_node::getName().c_str());
    green_light_ = true;
  }
  else
    ROS_ERROR("%s ---> FAILED <---", ros::this_node::getName().c_str());
}

void IdmindSensors::runPeriodically()
{
  frequency_ = 20.0;

  ros::Rate r(frequency_);
  while(n_.ok())
  {
    ros::spinOnce();

    if (reconnecting_ && hardstop_client_.exists())
    {
      hardstop_client_.shutdown();
      hardstop_client_ = n_.serviceClient<idmind_motors::Hardstop>("/idmind_motors/hardstop", true);
      ROS_INFO("Reconnected to hardstop service server.");
      reconnecting_ = false;
    }

    readPublishGroundSensors();

    r.sleep();
  }
}

bool IdmindSensors::batteriesService(idmind_sensors::Batteries::Request& req, idmind_sensors::Batteries::Response& res)
{
  std::vector<double> batteries(3, 0.0);
  res.success = readBatteries(&batteries);

  res.motors_battery = batteries[0];
  res.electronics_battery = batteries[1];
  res.charger_voltage = batteries[2];

  return res.success;
}

bool IdmindSensors::readBatteries(std::vector<double>* batteries)
{
  if (serial_.write((uint8_t)batteries_command_))
  {
    uint8_t buffer[7];
    if (serial_.read(buffer, 7, true) && buffer[0] == batteries_command_)
    {
      for (int i = 0; i < 3; i++)
        (*batteries)[i] = static_cast<double>(buffer[i+1]) / 10.0;

      return true;
    }
    else
      ROS_ERROR("%s --> Failed to read batteries.", ros::this_node::getName().c_str());
  }

  return false;
}

bool IdmindSensors::dockUndockService(idmind_sensors::DockUndock::Request& req, idmind_sensors::DockUndock::Response& res)
{
  // 1 - dock / 2 - undock & exit / 3 - enter
  if (req.control < 1 || req.control > 3)
    return false;

  int sum = 0;

  if (req.control == 1)
  {
    sum += sendCommandControl(0x45, 1);
    sum += sendCommandControl(0x40, 1);
    sum += sendCommandControl(0x41, 1);
    sum += sendCommandControl(0x46, 1);
  }
  else if (req.control == 2)
  {
    sum += sendCommandControl(0x45, 2);
    sum += sendCommandControl(0x46, 2);
    sum += sendCommandControl(0x40, 2);
    sum += sendCommandControl(0x41, 2);
  }

  if (req.control != 3 && sum != 4)
    return false;

  if (req.control == 1)
    return true;

  geometry_msgs::Twist twist;
  twist.linear.x = req.control == 2 ? 0.4 : -0.08;

  double wait_time = req.control == 2 ? 1.5 : 8.0;
  ros::Time now = ros::Time::now();

  while (ros::Time::now() < now + ros::Duration(wait_time))
  {
    twist_pub_.publish(twist);
    ros::Duration(1/frequency_).sleep();
  }

  twist.linear.x = 0.0;
  twist_pub_.publish(twist);
  ros::Duration(1.0).sleep();

  // Add voltage check!
  res.success = true;
  return true;
}

int IdmindSensors::sendCommandControl(int command, int control)
{
  uint8_t msg[2];
  msg[0] = (uint8_t)command;
  msg[1] = (uint8_t)control;

  if (serial_.write(msg, 2))
  {
    uint8_t buffer[4];
    if (!serial_.read(buffer, 4, true) || !(buffer[0] == command))
    {
      ROS_ERROR("%s --> Failed to send dock/undock 0x%X %d.", ros::this_node::getName().c_str(), command, control);
      return 0;
    }
  }

  ros::Duration(control == 2 ? 0.01 : 0.5).sleep();

  return 1;
}

void IdmindSensors::serviceCheck(const ros::TimerEvent&)
{
  if ((!hardstop_client_.isValid() || !hardstop_client_.exists()) && !reconnecting_)
  {
    ROS_INFO("Lost connection to hardstop service server. Reconnecting...");
    reconnecting_ = true;
  }
}

void IdmindSensors::sendHardstop()
{
  if (ros::Time::now() > last_hardstop_ + ros::Duration(2.0))
  {
    idmind_motors::Hardstop msg;
    msg.request.activate = 1;
    msg.request.hardstop_time = 2;
    msg.request.back_up = 0;
    msg.request.stay_blocked = 0;
    hardstop_client_.call(msg);
    last_hardstop_ = ros::Time::now();
  }
}

void IdmindSensors::readPublishGroundSensors()
{
  if (serial_.write((uint8_t)ground_sensors_command_))
  {
    uint8_t buffer[8];
    if (serial_.read(buffer, 8, true) && buffer[0] == ground_sensors_command_)
    {
      std_msgs::Int32MultiArray ground_sensors;
      ground_sensors.data.resize(4, 0);

      for (int i = 0; i < 4; i++)
      {
        ground_sensors.data[3-i] = static_cast<int>(buffer[i+1]); //Left to right.

        if (ground_sensors.data[3-i] < ground_min_[3-i])
//          ground_min_[3-i] = ground_sensors.data[3-i];
//          sendHardstop();
          ;
      }

//      ROS_INFO("%d | %d | %d | %d", ground_sensors.data[0], ground_sensors.data[1],
//          ground_sensors.data[2], ground_sensors.data[3]);

//      ROS_INFO("%d | %d | %d | %d", ground_min_[0], ground_min_[1],
//          ground_min_[2], ground_min_[3]);

      ground_sensors_pub_.publish(ground_sensors);
    }
    else
      ROS_ERROR("%s --> Failed to read ground sensors.", ros::this_node::getName().c_str());
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "idmind_sensors");

  IdmindSensors sensors;

  if (sensors.green_light_)
    sensors.runPeriodically();

  return 0;
}
