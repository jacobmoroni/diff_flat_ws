#include <controller_old.h>
#include <stdio.h>

namespace controller_old
{

ControllerOld::ControllerOld() :
  nh_(ros::NodeHandle()),
  nh_private_("~")
{
  // retrieve global MAV params (mass and max thrust)
  ros::NodeHandle nh_mav(ros::this_node::getNamespace());
  mass_ = nh_mav.param<double>("dynamics/mass", 2.856);
  max_thrust_ = nh_mav.param<double>("dynamics/max_F", 59.844);
  drag_constant_ = nh_mav.param<double>("linear_mu", 0.1);
  thrust_eq_= (9.80665 * mass_) / max_thrust_;
  is_flying_ = false;

  // max_.roll = nh_private_.param<double>("max_roll", 0.15);
  // max_.pitch = nh_private_.param<double>("max_pitch", 0.15);
  max_.roll = nh_private_.param<double>("max_roll", 1.0);
  max_.pitch = nh_private_.param<double>("max_pitch", 1.0);
  max_.yaw_rate = nh_private_.param<double>("max_yaw_rate", 45.0*M_PI/180.0);
  max_.throttle = nh_private_.param<double>("max_throttle", 1.0);
  max_.u = nh_private_.param<double>("max_u", 1.0);
  max_.v = nh_private_.param<double>("max_v", 1.0);
  max_.w = nh_private_.param<double>("max_w", 1.0);

  _func = boost::bind(&ControllerOld::reconfigure_callback, this, _1, _2);
  _server.setCallback(_func);

  // Set up Publishers and Subscriber
  state_sub_ = nh_.subscribe("estimate", 1, &ControllerOld::stateCallback, this);
  is_flying_sub_ = nh_.subscribe("is_flying", 1, &ControllerOld::isFlyingCallback, this);
  cmd_sub_ = nh_.subscribe("high_level_command", 1, &ControllerOld::cmdCallback, this);
  uff_sub_ = nh_.subscribe("diff_flat_cmd", 1, &ControllerOld::u_ffCallback, this);

  command_pub_ = nh_.advertise<rosflight_msgs::Command>("command", 1);
}


void ControllerOld::stateCallback(const nav_msgs::OdometryConstPtr &msg)
{
  static double prev_time = 0;
  if(prev_time == 0)
  {
    prev_time = msg->header.stamp.toSec();
    return;
  }

  // Calculate time
  double now = msg->header.stamp.toSec();
  double dt = now - prev_time;
  prev_time = now;

  if(dt <= 0)
    return;

  // This should already be coming in NED
  xhat_.pn = msg->pose.pose.position.x;
  xhat_.pe = msg->pose.pose.position.y;
  xhat_.pd = msg->pose.pose.position.z;

  xhat_.u = msg->twist.twist.linear.x;
  xhat_.v = msg->twist.twist.linear.y;
  xhat_.w = msg->twist.twist.linear.z;

  // Convert Quaternion to RPY
  tf::Quaternion tf_quat;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, tf_quat);
  tf::Matrix3x3(tf_quat).getRPY(xhat_.phi, xhat_.theta, xhat_.psi);
  xhat_.theta = xhat_.theta;
  xhat_.psi = xhat_.psi;

  xhat_.p = msg->twist.twist.angular.x;
  xhat_.q = msg->twist.twist.angular.y;
  xhat_.r = msg->twist.twist.angular.z;

  if(is_flying_)
  {
    computeControl(dt);
    publishCommand();
  }
  else
  {
    resetIntegrators();
    prev_time_ = msg->header.stamp.toSec();
  }
  }


void ControllerOld::isFlyingCallback(const std_msgs::BoolConstPtr &msg)
{
  is_flying_ = msg->data;
}


void ControllerOld::cmdCallback(const rosflight_msgs::CommandConstPtr &msg)
{
  switch(msg->mode)
  {
    case rosflight_msgs::Command::MODE_XPOS_YPOS_YAW_ALTITUDE:
      xc_.pn = msg->x;
      xc_.pe = msg->y;
      xc_.pd = msg->F;
      xc_.psi = msg->z;
      control_mode_ = msg->mode;
      break;
    case rosflight_msgs::Command::MODE_XVEL_YVEL_YAWRATE_ALTITUDE:
      xc_.u = msg->x;
      xc_.v = msg->y;
      xc_.pd = msg->F;
      xc_.r = msg->z;
      control_mode_ = msg->mode;
      break;
    case rosflight_msgs::Command::MODE_XACC_YACC_YAWRATE_AZ:
      xc_.ax = msg->x;
      xc_.ay = msg->y;
      xc_.az = msg->F;
      xc_.r = msg->z;
      control_mode_ = msg->mode;
      break;
    default:
      ROS_ERROR("roscopter/controller: Unhandled command message of type %d", msg->mode);
      break;
  }
}

void ControllerOld::u_ffCallback(const rosflight_msgs::CommandConstPtr &msg)
{
    xc_.p_r = msg->x;
    xc_.th_r = msg->y;
    xc_.T_r = msg->F;
    // xc_.T_r = xc_.T_r*thrust_eq_;
    xc_.r_r = msg->z;
}

void ControllerOld::reconfigure_callback(roscopter::ControllerConfig &config, uint32_t level)
{
  double P, I, D, tau;
  tau = config.tau;
  P = config.x_dot_P;
  I = config.x_dot_I;
  D = config.x_dot_D;
  PID_u_.setGains(P, I, D, tau);

  P = config.y_dot_P;
  I = config.y_dot_I;
  D = config.y_dot_D;
  PID_v_.setGains(P, I, D, tau);

  P = config.z_dot_P;
  I = config.z_dot_I;
  D = config.z_dot_D;
  PID_w_.setGains(P, I, D, tau);

  P = config.north_P;
  I = config.north_I;
  D = config.north_D;
  PID_x_.setGains(P, I, D, tau);

  P = config.east_P;
  I = config.east_I;
  D = config.east_D;
  PID_y_.setGains(P, I, D, tau);

  P = config.down_P;
  I = config.down_I;
  D = config.down_D;
  PID_z_.setGains(P, I, D, tau);

  P = config.psi_P;
  I = config.psi_I;
  D = config.psi_D;
  PID_psi_.setGains(P, I, D, tau);

  max_.roll = config.max_roll;
  max_.pitch = config.max_pitch;
  max_.yaw_rate = config.max_yaw_rate;
  max_.throttle = config.max_throttle;
  max_.u = config.max_n_dot;
  max_.v = config.max_e_dot;
  max_.w = config.max_d_dot;

  // ROS_INFO("new gains");

  resetIntegrators();
}


void ControllerOld::computeControl(double dt)
{
  if(dt <= 0.0000001)
  {
    // This messes up the derivative calculation in the PID controllers
    return;
  }

  uint8_t mode_flag = control_mode_;

  if(mode_flag == rosflight_msgs::Command::MODE_XPOS_YPOS_YAW_ALTITUDE)
  {
    // Figure out desired velocities (in inertial frame)
    // By running the position controllers
    double pndot_c = PID_x_.computePID(xc_.pn, xhat_.pn, dt);
    double pedot_c = PID_y_.computePID(xc_.pe, xhat_.pe, dt);

    // Calculate desired yaw rate
    // First, determine the shortest direction to the commanded psi
    if(fabs(xc_.psi + 2*M_PI - xhat_.psi) < fabs(xc_.psi - xhat_.psi))
    {
      xc_.psi += 2*M_PI;
    }
    else if (fabs(xc_.psi - 2*M_PI -xhat_.psi) < fabs(xc_.psi - xhat_.psi))
    {
      xc_.psi -= 2*M_PI;
    }
    xc_.r = saturate(PID_psi_.computePID(xc_.psi, xhat_.psi, dt), max_.yaw_rate, -max_.yaw_rate);                               //Added in u_ff +xc_.r_r

    // Rotate into body frame
    /// TODO: Include pitch and roll in this mapping
    xc_.u = saturate(pndot_c*cos(xhat_.psi) + pedot_c*sin(xhat_.psi), max_.u, -1.0*max_.u);
    xc_.v = saturate(-pndot_c*sin(xhat_.psi) + pedot_c*cos(xhat_.psi), max_.v, -1.0*max_.v);

    mode_flag = rosflight_msgs::Command::MODE_XVEL_YVEL_YAWRATE_ALTITUDE;
  }

  if(mode_flag == rosflight_msgs::Command::MODE_XVEL_YVEL_YAWRATE_ALTITUDE)
  {
    double max_ax = sin(acos(thrust_eq_));
    double max_ay = sin(acos(thrust_eq_));
    xc_.ax = saturate(PID_u_.computePID(xc_.u, xhat_.u, dt) + drag_constant_*xhat_.u /(9.80665 * mass_), max_ax, -max_ax);        //Added in u_ff +xc_.p_r
    xc_.ay = saturate(PID_v_.computePID(xc_.v, xhat_.v, dt) + drag_constant_*xhat_.v /(9.80665 * mass_), max_ay, -max_ay);       //Added in u_ff +xc_.th_r

    // Nested Loop for Altitude
    double pddot = -sin(xhat_.theta) * xhat_.u + sin(xhat_.phi)*cos(xhat_.theta)*xhat_.v + cos(xhat_.phi)*cos(xhat_.theta)*xhat_.w;
    double pddot_c = saturate(PID_w_.computePID(xc_.pd, xhat_.pd, dt, pddot), max_.w, -max_.w);
    xc_.az = PID_z_.computePID(pddot_c, pddot, dt);                                                         //Added in u_ff +xc_.T_r
    //ROS_INFO("pddot = %f, pddot_c = %f, az_c = %f", pddot, pddot_c, xc_.az);
    mode_flag = rosflight_msgs::Command::MODE_XACC_YACC_YAWRATE_AZ;
  }

  if(mode_flag == rosflight_msgs::Command::MODE_XACC_YACC_YAWRATE_AZ)
  {
    // Model inversion (m[ax;ay;az] = m[0;0;g] + R'[0;0;-T]
    // This model tends to pop the MAV up in the air when a large change
    // in control is commanded as the MAV rotates to it's commanded attitude while also ramping up throttle.
    // It works quite well, but it is a little oversimplified.
    double total_acc_c = sqrt((1.0-xc_.az)*(1.0-xc_.az) + xc_.ax*xc_.ax + xc_.ay*xc_.ay); // (in g's)
    //ROS_INFO("total_acc = %f", total_acc_c);
    if (total_acc_c > 0.001)
    {
      xc_.phi = asin(xc_.ay / total_acc_c);
      xc_.theta = -1.0*asin(xc_.ax / total_acc_c);
    }
    else
    {
      xc_.phi = 0;
      xc_.theta = 0;
    }

    // Calculate actual throttle (saturate az to be falling at 1 g)
    double max_az = 1.0 / thrust_eq_;
    xc_.az = saturate(xc_.az, 1.0, -max_az);
    total_acc_c = sqrt((1.0-xc_.az)*(1.0-xc_.az) + xc_.ax*xc_.ax + xc_.ay*xc_.ay); // (in g's)
    xc_.throttle = total_acc_c*thrust_eq_; // calculate the total thrust in normalized units

    //ROS_INFO("xc_.az = %f, max_az = %f, total_acc_c = %f, throttle = %f", xc_.az, max_az, total_acc_c, xc_.throttle);

    mode_flag = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
  }

  if(mode_flag == rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE)
  {
    // Pack up and send the command
    // With Feed Forward
    command_.mode = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
    command_.F = saturate(xc_.T_r, max_.throttle, 0.0);//+ xc_.T_r;
    command_.x = saturate(xc_.phi+ xc_.p_r, max_.roll, -max_.roll);//+ xc_.p_r;
    command_.y = saturate(xc_.theta+ xc_.th_r, max_.pitch, -max_.pitch);//+ xc_.th_r;
    command_.z = saturate(xc_.r+ xc_.r_r, max_.yaw_rate, -max_.yaw_rate);//+ xc_.r_r;

    // without feed forward
    // command_.F = saturate(xc_.throttle, max_.throttle, 0.0);//+ xc_.T_r;
    // command_.x = saturate(xc_.phi, max_.roll, -max_.roll);//+ xc_.p_r;
    // command_.y = saturate(xc_.theta, max_.pitch, -max_.pitch);//+ xc_.th_r;
    // command_.z = saturate(xc_.r, max_.yaw_rate, -max_.yaw_rate);//+ xc_.r_r;

    // ROS_INFO("F = %f", command_.F);
  }
}

void ControllerOld::publishCommand()
{
  command_pub_.publish(command_);
}

void ControllerOld::resetIntegrators()
{
  PID_u_.clearIntegrator();
  PID_v_.clearIntegrator();
  PID_x_.clearIntegrator();
  PID_y_.clearIntegrator();
  PID_z_.clearIntegrator();
  PID_psi_.clearIntegrator();
}

double ControllerOld::saturate(double x, double max, double min)
{
  x = (x > max) ? max : x;
  x = (x < min) ? min : x;
  return x;
}

double ControllerOld::sgn(double x)
{
  return (x >= 0.0) ? 1.0 : -1.0;
}

} // namespace controller
