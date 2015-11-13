#include "idmind_teleoperation/idmind_teleoperation.h"

IdmindTeleoperation::IdmindTeleoperation()
    : n_("~"), zeroes_(0), latched_counter_(0), autonomous_(true), latched_(false), publishing_(false), reconnecting_(false)
{
  joy_sub_ = n_.subscribe<sensor_msgs::Joy>("/joy", 1, &IdmindTeleoperation::joyCallback, this);
  twist_pub_ = n_.advertise<geometry_msgs::Twist>("/idmind_motors/twist", 1);
  chatter_pub_ = n_.advertise<std_msgs::String>("chatter", 50);

  hardstop_client_ = n_.serviceClient<idmind_motors::Hardstop>("/idmind_motors/hardstop", true);
//  hardstop_client_.waitForExistence();

  service_check_ = n_.createTimer(ros::Duration(0.1), &IdmindTeleoperation::serviceCheck, this);

  ROS_INFO("\e[32m%s ---> SUCCESSFULL <---\e[0m", ros::this_node::getName().c_str());
}

void IdmindTeleoperation::runPeriodically()
{
  ros::Rate r(80.0);
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

    if (!autonomous_ && publishing_ && ros::Time::now() > (last_received_ + ros::Duration(0.015))) // --> 12.5ms TODO turtlebot_teleop_key @ 30Hz
    {
      ROS_WARN("%s --> Joystick lagging/stopped.", ros::this_node::getName().c_str());
      twist_.linear.x = 0;
      twist_.linear.y = 0;
      twist_.angular.z = 0;
      twist_pub_.publish(twist_);

      publishing_ = false;
    }

    if (!autonomous_ && latched_ && publishing_)
      twist_pub_.publish(twist_);

    r.sleep();
  }
}

void IdmindTeleoperation::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  last_received_ = ros::Time::now();

  ++latched_counter_;

  if (latched_counter_ == 4)
  {
    latched_ = true;

    bool trust_button = last_received_ > last_button_ + ros::Duration(0.5) ? true : false;

    twist_.linear.x = msg->axes[3];
    twist_.linear.y = msg->axes[0];
    twist_.angular.z = msg->axes[2];

    if (twist_.linear.x == 0 &&
        twist_.linear.y == 0 &&
        twist_.angular.z == 0)
    {
      if (publishing_)
        zeroes_++;
    }
    else
      zeroes_ = 0;

    publishing_ = zeroes_ < 2 ? true : false;

    if (trust_button)
    {
      if (msg->buttons[0])
      {
        idmind_motors::Hardstop stop;
        stop.request.activate = 1;
        stop.request.hardstop_time = 2;
        stop.request.back_up = 0;
        stop.request.stay_blocked = 0;
        hardstop_client_.call(stop);
        last_button_ = ros::Time::now();
      }
      else
      {
        std_msgs::String chat;

        if (msg->buttons[1] && msg->buttons[4] && msg->buttons[5])
        {
          chat.data = "move";
          autonomous_ = true;
        }

        else if(msg->buttons[2])
        {
          chat.data = "stop";
          autonomous_ = false;
        }

        else if(msg->buttons[8] && msg->buttons[4] && msg->buttons[5])
          chat.data = "sleep";

        else if(msg->buttons[9] && msg->buttons[4] && msg->buttons[5])
          chat.data = "wake";

        if (!chat.data.empty())
        {
          chatter_pub_.publish(chat);
          last_button_ = ros::Time::now();
        }
      }
    }

    latched_counter_ = 0;
  }
  else
    latched_ = false;
}

void IdmindTeleoperation::serviceCheck(const ros::TimerEvent&)
{
  if ((!hardstop_client_.isValid() || !hardstop_client_.exists()) && !reconnecting_)
  {
    ROS_INFO("Lost connection to hardstop service server. Reconnecting...");
    reconnecting_ = true;
  }
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "idmind_teleoperation");

  IdmindTeleoperation teleoperation;
  teleoperation.runPeriodically();

  return 0;
}
