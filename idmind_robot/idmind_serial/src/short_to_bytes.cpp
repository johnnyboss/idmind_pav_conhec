#include <ros/ros.h>

void conversion(int value)
{
  unsigned char high_byte, low_byte;
//  high_byte = (unsigned char)(value >> 8);
//  low_byte = (unsigned char)(value & 0xFF);
  high_byte = (unsigned char)((value >> 8) & 0xFF);
  low_byte = (unsigned char)(value & 0xFF);
//  high_byte = (unsigned char)((value & 0xFF00) >> 8);
//  low_byte = (unsigned char)(value & 0x00FF);

//  int result = (short)((high_byte << 8) | low_byte);
//  int result = (short)(high_byte << 8) + (short)low_byte;
  int result = (short)((high_byte << 8) | (low_byte & 0xFF));
  if (result > 32767)  result = result - 65536;

  ROS_INFO("%d --> %u %u --> %d", value, high_byte, low_byte, result);
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "short_to_bytes");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);

  conversion(0);
  conversion(1);
  conversion(-1);
  conversion(100);
  conversion(-100);
  conversion(255);
  conversion(-255);
  conversion(256);
  conversion(-256);
  conversion(32767);
  conversion(-32768);

//  while (ros::ok())
//  {
//
//    ros::spinOnce();
//    loop_rate.sleep();
//  }

  return 0;
}
