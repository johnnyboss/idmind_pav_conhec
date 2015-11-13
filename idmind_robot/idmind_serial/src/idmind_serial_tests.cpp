#include <ros/ros.h>
#include "serial/serial.h"

short readShort(unsigned char *bytes)
{
  short result = (short)((*bytes << 8) | (*(bytes + 1) & 0xFF));
  if (result > 32767)  result = result - 65536;
  return result;
}

void write(serial::Serial &connection, short value)
{
  unsigned char msg[2];
  msg[0] = (unsigned char)((value >> 8) & 0xFF);
  msg[1] = (unsigned char)(value & 0xFF);
  connection.write(msg, 2);
  ROS_INFO("Sent: %d", value);
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "idmind_serial_tests");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);

  std::string my_port = "/dev/ttyACM1";
  uint32_t baudrate = 115200;
  uint32_t timeout = 5;

  serial::Serial serial_connection(my_port, baudrate, serial::Timeout::simpleTimeout(timeout));

  if (serial_connection.isOpen())
  {
    unsigned char buffer[2];
    //unsigned char *buffer_ptr = &buffer[0];
    int value;
    ros::Duration(2).sleep(); // Arduino UNO (Leonardo not necessary)
    serial_connection.flush();

    write(serial_connection, 0);
    write(serial_connection, 1);
    write(serial_connection, -1);
    write(serial_connection, 100);
    write(serial_connection, -100);
    write(serial_connection, 255);
    write(serial_connection, -255);
    write(serial_connection, 256);
    write(serial_connection, -256);
    write(serial_connection, 32767);
    write(serial_connection, -32768);

    while (ros::ok())
    {
      if (serial_connection.available() > 1)
      {
        serial_connection.read(buffer, 2);
//        serial_connection.read(&buffer[0], 2);
//        value = (short)((buffer[0] << 8) | (buffer[1] & 0xFF));
//        if (value > 32767)  value = value - 65536;
        value = readShort(buffer);
        ROS_INFO("Received: %d", value);
      }

      loop_rate.sleep();
    }
  }
  return 0;
}
