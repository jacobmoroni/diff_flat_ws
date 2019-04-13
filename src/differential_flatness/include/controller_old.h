#ifndef CONTROLLER_OLD_H
#define CONTROLLER_OLD_H

#include <ros/ros.h>
#include <rosflight_msgs/Command.h>
#include <rosflight_utils/simple_pid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include <stdint.h>
#include <dynamic_reconfigure/server.h>
#include <roscopter/ControllerConfig.h>

namespace controller_old
{

typedef struct
{
  double pn;
  double pe;
  double pd;

  double phi;
  double theta;
  double psi;

  double u;
  double v;
  double w;

  double p;
  double q;
  double r;

  double ax;
  double ay;
  double az;

  double p_r;
  double th_r;
  double T_r;
  double r_r;

  double throttle;
}state_t;

typedef struct
{
  double roll;
  double pitch;
  double yaw_rate;
  double throttle;
  double u;
  double v;
  double w;
} max_t;

class ControllerOld
{

public:

  ControllerOld();

private:

  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publishers and Subscribers
  ros::Subscriber state_sub_;
  ros::Subscriber is_flying_sub_;
  ros::Subscriber cmd_sub_;
  ros::Subscriber uff_sub_;

  ros::Publisher command_pub_;

  // Paramters
  double thrust_eq_;
  double mass_;
  double max_thrust_;
  double drag_constant_;
  bool is_flying_;

  // PID Controllers
  rosflight_utils::SimplePID PID_u_;
  rosflight_utils::SimplePID PID_v_;
  rosflight_utils::SimplePID PID_w_;
  rosflight_utils::SimplePID PID_x_;
  rosflight_utils::SimplePID PID_y_;
  rosflight_utils::SimplePID PID_z_;
  rosflight_utils::SimplePID PID_psi_;

  // Dynamic Reconfigure Hooks
  dynamic_reconfigure::Server<roscopter::ControllerConfig> _server;
  dynamic_reconfigure::Server<roscopter::ControllerConfig>::CallbackType _func;
  void reconfigure_callback(roscopter::ControllerConfig &config, uint32_t level);

  // Memory for sharing information between functions
  state_t xhat_ = {}; // estimate
  max_t max_ = {};
  rosflight_msgs::Command command_;
  state_t xc_ = {}; // command
  double prev_time_;
  uint8_t control_mode_;

  // Functions
  void stateCallback(const nav_msgs::OdometryConstPtr &msg);
  void isFlyingCallback(const std_msgs::BoolConstPtr &msg);
  void cmdCallback(const rosflight_msgs::CommandConstPtr &msg);
  void u_ffCallback(const rosflight_msgs::CommandConstPtr &msg);

  void computeControl(double dt);
  void resetIntegrators();
  void publishCommand();
  double saturate(double x, double max, double min);
  double sgn(double x);
};
}

#endif
