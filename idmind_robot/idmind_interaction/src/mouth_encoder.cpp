#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>

class MouthEncoder
{
public:
  MouthEncoder();
//  ~MouthEncoder();

private:
  void mouthShapeCallback(const std_msgs::UInt8::ConstPtr& msg);

  void readMouths();

  ros::NodeHandle n_;
  ros::Subscriber mouth_shape_sub_;
  ros::Publisher encoded_mouth_pub_;

  std_msgs::UInt8MultiArray zeros_;
  std::vector<std_msgs::UInt8MultiArray> mouths_;

  std::string idmind_ns_;

  int number_of_mouths_;
};

MouthEncoder::MouthEncoder()
    : n_("~"), idmind_ns_("idmind_robot/")
{
  mouth_shape_sub_ = n_.subscribe<std_msgs::UInt8>("mouth_shape", 10, &MouthEncoder::mouthShapeCallback, this);
  encoded_mouth_pub_ = n_.advertise<std_msgs::UInt8MultiArray>("/idmind_interaction/mouth", 10);

  ros::param::param<int>(idmind_ns_+"mouth_command", number_of_mouths_, 5);

  zeros_.data.resize(32, 0);
  mouths_.resize(number_of_mouths_+1, zeros_);

  readMouths();
}

void MouthEncoder::mouthShapeCallback(const std_msgs::UInt8::ConstPtr& msg)
{
  if ((*msg).data <= number_of_mouths_)
    encoded_mouth_pub_.publish(mouths_[(*msg).data]);
}

void MouthEncoder::readMouths()
{
  std::string homedir = getenv("HOME");
  std::ifstream my_file((homedir + std::string("/catkin_ws/src/idmind_robot/no_build/config/behaviours/mouths.txt")).c_str());

  if (!my_file)
    ROS_ERROR("%s --> Failed to open mouths file.", ros::this_node::getName().c_str());

  std::string my_string;

  for (int i=0; i<=number_of_mouths_; i++)
  {
    std::getline(my_file, my_string);
    std::getline(my_file, my_string);

    for (int j=0; j<8; j++)
    {
      std::getline(my_file, my_string);
      my_string.erase(std::remove_if(my_string.begin(), my_string.end(), isspace), my_string.end());

      for (int k=0; k<32; k++)
      {
        if (my_string[k] == '0')
          mouths_[i].data[31-k] += pow(2, 7-j);
      }
    }

//    for (int d=0; d<32; d++)
//      ROS_INFO("%d ", mouths_[i].data[d]);
  }

  my_file.close();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "mouth_encoder");

  MouthEncoder mouth_encoder;

  ros::spin();

  return 0;
}
