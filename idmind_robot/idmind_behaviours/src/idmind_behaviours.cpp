#include "idmind_behaviours/idmind_behaviours.h"

IdmindBehaviours::IdmindBehaviours()
    : n_("~"), action_client_(n_, "move_base", false), current_position_(3, 0.0),
      behaviour_ns_("high_behavior/"), behaviours_folder_("/catkin_ws/src/idmind_robot/no_build/config/behaviours/"),
      goal_radius_(2.0), target_(0), tentatives_(0),
      green_light_(true), to_next_target_(false), cancelled_(true)
{
  amcl_pose_sub_ = n_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("head", 10, &IdmindBehaviours::amclPoseCallback, this);

  head_pub_ = n_.advertise<std_msgs::UInt8MultiArray>("/idmind_interaction/head", 10);
  mouth_pub_ = n_.advertise<std_msgs::UInt8>("/mouth_encoder/mouth_shape", 10);
  lights_pub_ = n_.advertise<std_msgs::UInt8MultiArray>("/idmind_interaction/lights", 10); //Put back to 10. Was 100. Check there's no problems.
  arms_pub_ = n_.advertise<std_msgs::UInt16MultiArray>("/idmind_arms/position", 10);

  ros::Time now = ros::Time::now();
  while(true)
  {
    if (head_pub_.getNumSubscribers() != 0 &&
        mouth_pub_.getNumSubscribers() != 0 &&
        lights_pub_.getNumSubscribers() != 0)
      break;

    if (ros::Time::now() > now + ros::Duration(1.0))
    {
      ROS_ERROR("%s --> One or more topics were not subscribed to.", ros::this_node::getName().c_str());
      green_light_ = false;
      break;
    }

    ros::Duration(0.010).sleep();
  }

  ros::param::param<double>(behaviour_ns_+"wake_up_time", wake_up_time_, 10.0);
  ros::param::param<double>(behaviour_ns_+"nap_time", nap_time_, 14.0);
  ros::param::param<double>(behaviour_ns_+"nap_duration", nap_duration_, 1.0);
  ros::param::param<double>(behaviour_ns_+"sleep_time", sleep_time_, 17.0);

  last_aborted_ = ros::Time::now() - ros::Duration(10.0);

  readFiles();

//  while (!action_client_.waitForServer(ros::Duration(5.0)))
//    ROS_INFO("%s --> Waiting for the move_base action server to come up.", ros::this_node::getName().c_str());
}

void IdmindBehaviours::runPeriodically() //Change from spaces to tabs in files!!!
{
//    sendBehaviour(2);
//    ros::Duration(3.0).sleep();
//
//    sendBehaviour(0);
//    ros::Duration(1.0).sleep();

  ros::Rate r(30.0);
  while (n_.ok())
  {
    ros::spinOnce();

    if (!green_light_ && !cancelled_)
    {
      action_client_.cancelGoal();
      cancelled_ = true;
    }
    else if (green_light_)
    {
      if (to_next_target_)
      {
        if (target_ == points_.size())
          target_ = 1;

        goal_.target_pose.header.frame_id = "map";
        goal_.target_pose.header.stamp =  ros::Time::now();
        goal_.target_pose.pose.position.x = points_[target_].x;
        goal_.target_pose.pose.position.y = points_[target_].y;
        goal_.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(points_[target_].th);

        action_client_.sendGoal(goal_);
        to_next_target_ = false;
      }

      if (action_client_.getState() == actionlib::SimpleClientGoalState::ABORTED && ros::Time::now() > last_aborted_ + ros::Duration(3.0))
      {
        last_aborted_ = ros::Time::now();
        ROS_INFO("%s --> Aborted, trying again.", ros::this_node::getName().c_str());

        tentatives_++;
        to_next_target_ = true;

        if (tentatives_ > 3 && target_ != 0)
        {
          ROS_INFO("%s --> Tried 3 times, switching to next one.", ros::this_node::getName().c_str());
          target_++;
          tentatives_ = 0;
        }
      }

      if (points_[target_].stop != 0 && action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        action_client_.cancelGoal();
        ROS_INFO("%s --> Reached target %d, will stop and do something.", ros::this_node::getName().c_str(), target_);

        sendBehaviour(points_[target_].behaviour);

        if (points_[target_].media != 0) // Behaviour chain? Media + beh + beh + media + beh + media + media...
          playMedia(points_[target_].media); // s->sleep m->media b->behaviour ... s1 m3 s2 m1 s2 b3 b1 s3

        if (points_[target_].stop == 2)
          green_light_ = false;

        target_++;
        to_next_target_ = true;
      }
      else if (points_[target_].stop == 0)
      {
        double dx = current_position_[0] - goal_.target_pose.pose.position.x;
        double dy = current_position_[1] - goal_.target_pose.pose.position.y;

        if  (sqrt(pow(dx, 2) + pow(dy, 2)) < goal_radius_)
        {
          ROS_INFO("%s --> Reached target %d, will go for next one.", ros::this_node::getName().c_str(), target_);

          target_++;
          to_next_target_ = true;
        }
      }
    }

    r.sleep();
  }
}

void IdmindBehaviours::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  current_position_[0] = (*msg).pose.pose.position.x;
  current_position_[1] = (*msg).pose.pose.position.y;
}

void IdmindBehaviours::playMedia(int i)
{
  system((std::string("/home/monarch/catkin_ws/src/no_build/r_s/") + media_[i].type + std::string(".sh") + media_[i].file_name).c_str());
}

void IdmindBehaviours::sendBehaviour(int i)
{
  std_msgs::UInt8MultiArray head;
  head.data.resize(2);
  head.data[0] = behaviours_[i].head;
  head.data[1] = 40;
  head_pub_.publish(head);

  std_msgs::UInt8 mouth;
  mouth.data = behaviours_[i].mouth;
  mouth_pub_.publish(mouth);

  for (int j=0; j<6; j++)
  {
    std_msgs::UInt8MultiArray lights;
    lights.data.resize(5);
    lights.data[0] = j;
    lights.data[1] = colours_[behaviours_[i].leds[j]].red;
    lights.data[2] = colours_[behaviours_[i].leds[j]].green;
    lights.data[3] = colours_[behaviours_[i].leds[j]].blue;
    lights.data[4] = 20;
    lights_pub_.publish(lights);
  }

  std_msgs::UInt16MultiArray arms;
  arms.data.resize(4);
  arms.data[0] = behaviours_[i].left_arm;
  arms.data[1] = 1500;
  arms.data[2] = behaviours_[i].right_arm;
  arms.data[3] = 1500;
  arms_pub_.publish(arms);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "idmind_behaviours");

  IdmindBehaviours behaviours;

  if (behaviours.green_light_)
    behaviours.runPeriodically();

  return 0;
}
