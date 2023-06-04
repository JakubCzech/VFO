#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

class vfo : public rclcpp::Node
{
private:
  double phi;
  double sigma;
  double Kp;
  double Ka;
  double ksid;
  double v_d;
  double max_speed;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

public:
  vfo();
  double norm(const std::array<double, 2>& v);
  double atan2c(double phi_current, double phi_last);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void publish_cmd_vel(double v, double w);
  void calculate_velocity(const double x, const double y, const double theta_d);
};

vfo::vfo() : Node("vfo")
{
  RCLCPP_INFO(this->get_logger(), "VFO node started");
  auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
  phi = 0.0;
  sigma = 1.0;
  Kp = 1.0;
  Ka = 2.0 * Kp;
  ksid = 1.0;
  v_d = 0.2;
  max_speed = 1.5;
  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", sensor_qos, std::bind(&vfo::odom_callback, this, std::placeholders::_1));
  cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

double vfo::norm(const std::array<double, 2>& v) 
{
   double sum_of_squares = 0.0;
    for (const auto& xi : v) {
        sum_of_squares += xi * xi;
    }
    return sqrt(sum_of_squares);
}
void vfo::calculate_velocity(const double x, const double y, const double theta_d)
{

  double F, Fx, Fxx, Fy, Fyy, Fxy;
// A = 1.0;
// B = 2.0;
// Sigma = 1;
// %--------------------------------------------------------------------------
// F = Sigma*(((X^2)/(A^2)) + ((Y^2)/(B^2)) - 1); %równanie elipsy na płaszczyźnie
// Fx = Sigma*2*X/(A^2);                          %pochodna cząstkowa po X
// Fxx = Sigma*2/(A^2);                           %druga pochodna cząstkowa po X
// Fy = Sigma*2*Y/(B^2);                          %pochodna cząstkowa po Y
// Fyy = Sigma*2/(B^2);                           %druga pochodna cząstkowa po Y
// Fxy = 0;                                       %druga pochodna mieszana 

  double A = 1.0;
  double B = 2.0;
  F = sigma * ((pow(x, 2) / pow(A, 2)) + (pow(y, 2) / pow(B, 2)) - 1);
  Fx = sigma * 2 * x / pow(A, 2);
  Fxx = sigma * 2 / pow(A, 2);
  Fy = sigma * 2 * y / pow(B, 2);
  Fyy = sigma * 2 / pow(B, 2);
  Fxy = 0;

  // F = sigma * (pow(x, 2) + pow((y / 2), 2) - 1);
  // Fx = sigma * 2 * x;
  // Fxx = sigma * 2;
  // Fy = sigma * y;
  // Fyy = sigma;
  // Fxy = 0;
  // VFO algorithm
  std::array<double, 2> NEGnablaF;
  NEGnablaF[0] = -Fx;
  NEGnablaF[1] = -Fy;
  
  double NnablaF = norm(NEGnablaF);

  std::array<double, 2> WersorNEGnablaF;
  WersorNEGnablaF[0] = NEGnablaF[0] / NnablaF;
  WersorNEGnablaF[1] = NEGnablaF[1] / NnablaF;

  std::array<std::array<double, 2>, 2> Rot;
  Rot[0][0] = 0;
  Rot[0][1] = 1;
  Rot[1][0] = -1;
  Rot[1][1] = 0;

  std::array<double, 2> NEGnablaFORT;
  NEGnablaFORT[0] = Rot[0][0] * NEGnablaF[0] + Rot[0][1] * NEGnablaF[1];
  NEGnablaFORT[1] = Rot[1][0] * NEGnablaF[0] + Rot[1][1] * NEGnablaF[1];

  std::array<double, 2> WersorNEGnablaFORT;
  WersorNEGnablaFORT[0] = NEGnablaFORT[0] / NnablaF;
  WersorNEGnablaFORT[1] = NEGnablaFORT[1] / NnablaF;

  std::array<double, 2> H;
  H[0] = v_d * WersorNEGnablaFORT[0] + Kp * F * WersorNEGnablaF[0];
  H[1] = v_d * WersorNEGnablaFORT[1] + Kp * F * WersorNEGnablaF[1];

  double u2 = H[0] * cos(theta_d) + H[1] * sin(theta_d);

  double tetaa = atan2(ksid * H[1], ksid * H[0]);

  tetaa = atan2c(tetaa, phi);

  phi = tetaa;

  double Etetaa = tetaa - theta_d;

  double xp = u2 * cos(theta_d);
  double yp = u2 * sin(theta_d);

  double dFdt = Fx * xp + Fy * yp;

  std::array<double, 2> dWersorNEGnablaFdt;
  dWersorNEGnablaFdt[0] = (Fy * ((Fx * Fxy - Fy * Fxx) * xp + (Fx * Fyy - Fy * Fxy) * yp)) / pow(sqrt(pow(Fx, 2) + pow(Fy, 2)), 3);
  dWersorNEGnablaFdt[1] = (Fx * ((Fy * Fxx - Fx * Fxy) * xp + (Fy * Fxy - Fx * Fyy) * yp)) / pow(sqrt(pow(Fx, 2) + pow(Fy, 2)), 3);
  
  std::array<double, 2> Hp;
  Hp[0] = v_d * Rot[0][0] * dWersorNEGnablaFdt[0] + v_d * Rot[0][1] * dWersorNEGnablaFdt[1] + Kp * (dFdt * WersorNEGnablaF[0] + F * dWersorNEGnablaFdt[0]);
  Hp[1] = v_d * Rot[1][0] * dWersorNEGnablaFdt[0] + v_d * Rot[1][1] * dWersorNEGnablaFdt[1] + Kp * (dFdt * WersorNEGnablaF[1] + F * dWersorNEGnablaFdt[1]);
  double tetaap = (Hp[1] * H[0] - H[1] * Hp[0]) / (pow(H[0], 2) + pow(H[1], 2));
  double u1 = Ka * Etetaa + tetaap;

  // BSP
  std::array<double, 2> S;
  S[0] = std::max(abs(u1) / max_speed, 1.0);
  S[1] = std::max(abs(u2) / max_speed, 1.0);
  u1 /= S[0];
  u2 /= S[1];
  // END OF BSP
  publish_cmd_vel(u1, u2);
}

void vfo::publish_cmd_vel(double v, double w)
{
  auto cmd_vel_msg = geometry_msgs::msg::Twist();
  cmd_vel_msg.linear.x = v;
  cmd_vel_msg.angular.z = w;
  cmd_vel_pub->publish(cmd_vel_msg);
}

void vfo::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  tf2::Quaternion quat(msg->pose.pose.orientation.x,
                       msg->pose.pose.orientation.y,
                       msg->pose.pose.orientation.z,
                       msg->pose.pose.orientation.w);

  RCLCPP_INFO(this->get_logger(), "x: %f, y: %f", x, y);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  calculate_velocity(x, y, yaw);
}

double vfo::atan2c(double phi_current, double phi_last)
{
  double delta_phi = phi_current - phi_last;
  if (delta_phi > M_PI)
  {
    phi_current -= 2 * M_PI;
  }
  else if (delta_phi < -M_PI)
  {
    phi_current += 2 * M_PI;
  }

  if (!std::isfinite(phi_current))
  {
    return 0.0;
  }
  else if (std::isnan(phi_current))
  {
    return 0.0;
  }
  return phi_current;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vfo>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
