#include "idmind_arms/idmind_arms.h"

IdmindArms::IdmindArms()
    : n_("~"), serial_("/dev/idmind-armsboard", 115200, 5), time_constant_(11.2),
      min_ticks_(100), max_ticks_(900), range_(150), mode_ON_(0x60), green_light_(false)
{
  position_sub_ = n_.subscribe<std_msgs::UInt16MultiArray>("position", 10, &IdmindArms::positionCallback, this);

//  head_angle_pub_ = n_.advertise<std_msgs::UInt8>("head_angle", 100);

  torque_serv_ = n_.advertiseService("torque", &IdmindArms::torqueService, this);

  position_.resize(2, 0);
  play_time_.resize(2, 0);
  last_position_.resize(2, -1);
  send_position_.resize(2, 0);
  mode_.resize(2, 0);

  serial_.check_sum_ = false;

  if (serial_.ready_)
  {
    ROS_INFO("\e[32m%s ---> SUCCESSFULL <---\e[0m", ros::this_node::getName().c_str());
    green_light_ = true;
  }
  else
    ROS_ERROR("%s ---> FAILED <---", ros::this_node::getName().c_str());
}

void IdmindArms::initialize()
{
  sendTorqueCommand(mode_ON_, mode_ON_);
  ros::Duration(0.010).sleep();
}

void IdmindArms::runPeriodically()
{
  ros::Rate r(20.0);
  while(n_.ok())
  {
    ros::spinOnce();

//    readTorque();
    ros::Duration(0.010).sleep();

    sendPositionCommand();

    r.sleep();
  }
}

void IdmindArms::shutdown()
{
  ros::Duration(0.005).sleep();
  sendTorqueCommand(0, 0);
}

void IdmindArms::positionCallback(const std_msgs::UInt16MultiArray::ConstPtr& msg)
{
  for (int i=0; i<2; i++)
  {
    position_[i] = (*msg).data[2*i];
    play_time_[i] = (*msg).data[2*i+1];

    if (position_[i] < 0) position_[i] = 0;
    if (position_[i] > range_) position_[i] = range_;

    if (play_time_[i] < 0) play_time_[i] = 0;
    if (play_time_[i] > 2500) position_[i] = 2500;

    play_time_[i] = static_cast<double>(play_time_[i]) / time_constant_;

    if (position_[i] != last_position_[i] && mode_[i] == mode_ON_)
      send_position_[i] = 1;
  }
}

void IdmindArms::sendPositionCommand()
{
  for (int i=0; i<2; i++)
  {
    if (send_position_[i])
    {
      int offset_ticks = i==0 ? min_ticks_:max_ticks_;
      double ticks_x_degree = static_cast<double>(max_ticks_ - min_ticks_) / range_;
      int angle_ticks = (i==0 ? 1:-1) * ticks_x_degree * position_[i] + offset_ticks;

      uint8_t angle_bytes[2];
      serial_.addInt(angle_bytes, angle_ticks);

      uint8_t command[12];
      command[0]= 0xFF;
      command[1]= 0xFF;
      command[2]= 12;
      command[3]= i;
      command[4] = 0x05;

      command[7] = angle_bytes[1];
      command[8] = angle_bytes[0];
      command[9] = 0x04;
      command[10] = i;
      command[11] = play_time_[i];

      command[5] = (command[2] ^ command[3] ^ command[4] ^ command[7] ^ command[8] ^ command[9] ^ command[10] ^ command[11]) & 0xFE;
      command[6] = (~command[5]) & 0xFE;

      if (!serial_.write(command, 12))
        ROS_ERROR("%s --> Failed to send arm %d position command.", ros::this_node::getName().c_str(), i);

//      ROS_INFO("Position: %d %d %d %d %d %d %d %d %d %d", command[2], command[3], command[4], command[5],
//        command[6], command[7], command[8], command[9], command[10], command[11]);

      last_position_[i] = position_[i];
      send_position_[i] = 0;
    }
  }
}

bool IdmindArms::torqueService(idmind_arms::Torque::Request& req, idmind_arms::Torque::Response& res)
{
  if (sendTorqueCommand(req.mode_left, req.mode_right))
  {
    res.success = true;
    return true;
  }

  return false;
}

bool IdmindArms::sendTorqueCommand(int mode_left, int mode_right)
{
  mode_[0] = mode_left;
  mode_[1] = mode_right;

  for (int i=0; i<2; i++)
    if (mode_[i] != mode_ON_)
      last_position_[i] = -1;

  int ID = 0x00;
  if (mode_left == mode_right)
    ID = 0xFE;

  while (true)
  {
    uint8_t command[10];
    command[0]= 0xFF;
    command[1]= 0xFF;
    command[2]= 10;
    command[3]= ID;
    command[4] = 0x03;

    command[7] = 52;
    command[8] = 1;

    if (ID == 0x00 || ID == 0xFE)
      command[9] = mode_left;
    else
      command[9] = mode_right;

    command[5] = (command[2] ^ command[3] ^ command[4] ^ command[7] ^ command[8] ^ command[9]) & 0xFE;
    command[6] = (~command[5]) & 0xFE;

    if (!serial_.write(command, 10))
    {
      ROS_ERROR("%s --> Failed to send arm %d torque command.", ros::this_node::getName().c_str(), ID);
      return false;
    }

//    ROS_INFO("Torque: %d %d %d %d %d %d %d %d", command[2], command[3], command[4], command[5],
//      command[6], command[7], command[8], command[9]);

    if (ID == 0x01 || ID == 0xFE) break;
    if (ID == 0x00) ID = 0x01;
  }

  return true;
}

int IdmindArms::readRAM(int ID, int memory, int number_bytes)
{
  uint8_t command[9];
  command[0]= 0xFF;
  command[1]= 0xFF;
  command[2]= 9;
  command[3]= ID;
  command[4] = 0x04;

  command[7] = memory;
  command[8] = number_bytes;

  command[5] = (command[2] ^ command[3] ^ command[4] ^ command[7] ^ command[8]) & 0xFE;
  command[6] = (~command[5]) & 0xFE;

  if (serial_.write(command, 9))
  {
    int size = 11 + number_bytes;
    uint8_t buffer[size];

    if (serial_.read(buffer, size, true))
    {
      int data;
      uint8_t array[2];

      array[0] = buffer[10];
      array[1] = buffer[9];

      if (number_bytes == 1)
        data = static_cast<int>(buffer[9]);
      else if (number_bytes == 2)
        data = serial_.readInt(array);

      return data;
    }
    else
      ROS_ERROR("%s --> Failed to read RAM %d from ID %d.", ros::this_node::getName().c_str(), memory, ID);
  }

  return -1;
}

void IdmindArms::readTorque()
{
  int position_left = readRAM(0, 64, 2);
  ros::Duration(0.005).sleep();
  int position_right = readRAM(1, 64, 2);
//  ROS_INFO("Left: %d | Right: %d", position_left, position_right);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "idmind_arms");

  IdmindArms arms;

  arms.initialize();

  if (arms.green_light_)
    arms.runPeriodically();

  arms.shutdown();


  return 0;
}
