#include "idmind_motors/idmind_motors.h"

void IdmindMotors::inverseKinematics(std::vector<int>* angular_vels)
{
  if (kinematics_.find("calibration") == 6) //kinematics_ = "wheel_calibration_1"
  {
    int wheel = atoi(kinematics_.substr(18).c_str());
    (*angular_vels)[wheel-1] = sign(twist_.linear.x)*velocity_offset_ + twist_.linear.x;
  }
  else if (kinematics_ == "differential")
  {
    std::vector<double> linear_vels(2, 0.0);

    linear_vels[0] = twist_.linear.x - twist_.angular.z * base_width_ / 2;
    linear_vels[1] = twist_.linear.x + twist_.angular.z * base_width_ / 2;

    if (robot_ == "2wd")
    {
      if (linear_vels[0] > 1e-6)
        linear_vels[0] = 2.625 + 241.868*linear_vels[0] + 519.616*pow(linear_vels[0], 2.0) - 268.012*pow(linear_vels[0], 3.0);
      else if (linear_vels[0] < -1e-6)
        linear_vels[0] = -2.113 + 248.765*linear_vels[0] - 557.908*pow(linear_vels[0], 2.0) - 323.284*pow(linear_vels[0], 3.0);

      if (linear_vels[1] > 1e-6)
        linear_vels[1] = 18.084 + 153.815*linear_vels[1] + 715.814*pow(linear_vels[1], 2.0) - 404.342*pow(linear_vels[1], 3.0);
      else if (linear_vels[1] < -1e-6)
        linear_vels[1] = -4.388 + 240.616*linear_vels[1] - 540.111*pow(linear_vels[1], 2.0) - 293.803*pow(linear_vels[1], 3.0);
    }

    (*angular_vels)[0] = -(sign(linear_vels[0])*velocity_offset_ + static_cast<int>(linear_vels[0] / wheel_radius_));
    (*angular_vels)[1] = sign(linear_vels[1])*velocity_offset_ + static_cast<int>(linear_vels[1] / wheel_radius_);
  }
  else if (kinematics_ == "omnidirectional")
  {
    std::vector<double> linear_vels(4, 0.0);

    double vx = twist_.linear.x;
    double vy = twist_.linear.y;
    double wz = twist_.angular.z;

    linear_vels[0] = (vx - vy - base_width_*wz);
    linear_vels[1] = (vx + vy + base_width_*wz);
    linear_vels[2] = (vx + vy - base_width_*wz);
    linear_vels[3] = (vx - vy + base_width_*wz);

    if (robot_ == "omni")
    {
      for (int i = 0; i < 4; i++)
        linear_vels[i] *= 29;
    }

    double a = 1.0 / wheel_radius_;

    (*angular_vels)[0] = -(sign(linear_vels[0])*velocity_offset_ + a * linear_vels[0]);
    (*angular_vels)[1] =  (sign(linear_vels[1])*velocity_offset_ + a * linear_vels[1]);
    (*angular_vels)[2] = -(sign(linear_vels[2])*velocity_offset_ + a * linear_vels[2]);
    (*angular_vels)[3] =  (sign(linear_vels[3])*velocity_offset_ + a * linear_vels[3]);
  }
}

void IdmindMotors::velocitySmoother()
{
  if ((twist_.linear.x  != last_twist_.linear.x) ||
      (twist_.linear.y  != last_twist_.linear.y) ||
      (twist_.angular.z != last_twist_.angular.z))
  {
    double vx_inc, vy_inc, w_inc, max_vx_inc, max_vy_inc, max_w_inc;

    vx_inc = twist_.linear.x - last_twist_.linear.x;
    max_vx_inc = ((vx_inc*twist_.linear.x * twist_.linear.x*last_twist_.linear.x > 0.0)?max_acc_v_:max_dec_v_)*(1/frequency_);

    vy_inc = twist_.linear.y - last_twist_.linear.y;
    max_vy_inc = ((vy_inc*twist_.linear.y * twist_.linear.y*last_twist_.linear.y > 0.0)?max_acc_v_:max_dec_v_)*(1/frequency_);

    w_inc = twist_.angular.z - last_twist_.angular.z;
    max_w_inc = ((w_inc*twist_.angular.z * twist_.angular.z*last_twist_.angular.z > 0.0)?max_acc_w_:max_dec_w_)*(1/frequency_);

    double mod_inc = sqrt(vx_inc*vx_inc + w_inc*w_inc);
    double mod_max_inc = sqrt(max_vx_inc*max_vx_inc + max_w_inc*max_w_inc);

    double inc_v = std::abs(vx_inc) / mod_inc;
    double inc_w = std::abs(w_inc) / mod_inc;
    double max_inc_v = max_vx_inc / mod_max_inc;
    double max_inc_w = max_w_inc / mod_max_inc;
    double theta = atan2(max_inc_w, max_inc_v) - atan2(inc_w, inc_v);

    if (theta < 0)
      max_vx_inc = max_w_inc * std::abs(vx_inc) / std::abs(w_inc);
    else
      max_w_inc = max_vx_inc * std::abs(w_inc) / std::abs(vx_inc);

//    ROS_INFO("%f | %f | %f", twist_.linear.x, vx_inc, max_vx_inc);

    if (std::abs(vx_inc) > max_vx_inc)
      twist_.linear.x  = last_twist_.linear.x  + sign(vx_inc)*max_vx_inc;

    if (std::abs(vy_inc) > max_vy_inc)
      twist_.linear.y  = last_twist_.linear.y  + sign(vy_inc)*max_vy_inc;

    if (std::abs(w_inc) > max_w_inc)
      twist_.angular.z = last_twist_.angular.z + sign(w_inc)*max_w_inc;

    last_twist_ = twist_;
  }
}
