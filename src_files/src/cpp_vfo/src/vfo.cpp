#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

class vfo : public rclcpp::Node
{
private:
  // Common VFO
  double x, y, theta;
  double u1, u2;
  double last_Phi;
  double Kp, Ka, max_speed;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  //  Path VFO
  double sigma, ksid, v_d, u2d;
  double F, Fx, Fxx, Fy, Fyy, Fxy;
  // Point VFO
  double e_x0 = 0.0, e_y0 = 0.0;
  double e_x0_d = 0.0;

public:
  vfo();
  double atan2c(double y, double x);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void publish_cmd_vel();
  void VFO();
  void VFO_point(double x_d, double y_d, double theta_d);
  void BSP20();
  void get_function();

  double sgn(double v);
};

vfo::vfo() : Node("vfo")
{
  RCLCPP_INFO(this->get_logger(), "VFO node started");
  auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

  this->declare_parameter<double>("sigma", 1.0);
  this->declare_parameter<double>("Kp", 1.0);
  this->declare_parameter<double>("Ka", 4.0);
  this->declare_parameter<double>("ksid", 1.0);
  this->declare_parameter<double>("v_d", -0.25);
  this->declare_parameter<double>("max_speed", 1.5);
  this->get_parameter("sigma", sigma);
  this->get_parameter("Kp", Kp);
  this->get_parameter("Ka", Ka);
  this->get_parameter("ksid", ksid);
  this->get_parameter("v_d", v_d);
  this->get_parameter("max_speed", max_speed);

  RCLCPP_INFO(this->get_logger(), "sigma: %f, Kp: %f, Ka: %f, ksid: %f, v_d: %f, max_speed: %f", sigma, Kp, Ka, ksid, v_d, max_speed);
  sleep(5);
  last_Phi = 0.0;
  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", sensor_qos, std::bind(&vfo::odom_callback, this, std::placeholders::_1));
  cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

void vfo::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  x = msg->pose.pose.position.x;
  y = msg->pose.pose.position.y;
  tf2::Quaternion quat(msg->pose.pose.orientation.x,
                       msg->pose.pose.orientation.y,
                       msg->pose.pose.orientation.z,
                       msg->pose.pose.orientation.w);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  theta = yaw;
  // get_function();
  // VFO();

  VFO_point(16.0, 6.0, 1.0);
  BSP20();
  publish_cmd_vel();
}

void vfo::VFO()
{
  u2d = 0.25;
  // ||∇F (p)||
  double delta_F_p = sqrt(pow(Fx, 2) + pow(Fy, 2));
  // VFO algorithm
  // Common VFO
  // double e_x = x_d - x;
  // double e_y = y_d - y;
  // double e_mod = sqrt(pow(e_x, 2) + pow(e_y, 2));
  // End Common VFO

  // Równanie 9
  double w_px = -Fx;
  double w_py = -Fy;
  double w2_px = -Fy;
  double w2_py = Fx;
  // Równanie 10
  //  H = v_d * w⊥(p) / ||∇F(p)|| + Kp * F * w(p) / ||∇F(p)||
  // double h_x = v_d * w2_px / delta_F_p + Kp * F * w_px / delta_F_p;
  // double h_y = v_d * w2_py / delta_F_p + Kp * F * w_py / delta_F_p;
  double e_x;
  double e_y;
  double e_mod;
  double v_x;
  double v_y;
  // Common VFO
  double h_x = (Kp * e_x) + v_x;
  double h_y = (Kp * e_y) + v_y;
  u2 = h_x * cos(theta) + h_y * sin(theta);
  // End Common VFO
  // θa(p) = Atan2c ( ksid * h_y, ksid * h_x)
  // double theta_a = atan2(ksid * h_y, ksid * h_x);

  // and η1 = FxFxy − FyFxx , η2 = FxFyy − FyFxy , η3 = FyFxx − FxFxy ,η4 = FyFxy −FxFyy .
  double n1, n2, n3, n4;
  n1 = Fx * Fxy - Fy * Fxx;
  n2 = Fx * Fyy - Fy * Fxy;
  n3 = Fy * Fxx - Fx * Fxy;
  n4 = Fy * Fxy - Fx * Fyy;

  double v_xn = u2 / pow(delta_F_p, 3) * (Fy * (n1 * cos(theta) + n2 * sin(theta)));
  double v_yn = u2 / pow(delta_F_p, 3) * (Fx * (n3 * cos(theta) + n4 * sin(theta)));
  // Fn (Fx cθ + Fy sθ )(hx cθ + hy sθ )
  double F_n = (Fx * cos(theta) + Fy * sin(theta)) * (h_x * cos(theta) + h_y * sin(theta));
  v_x = -Fx / delta_F_p;
  v_y = -Fy / delta_F_p;
  // vdϑ˙yn(q)+kp[F˙n(q)ϑx (p) + F (p)ϑ˙xn(q)]
  double h_x_dot = v_d * v_yn + Kp * (F_n * v_x + F * v_xn);
  // −vdϑ˙xn(q)+kp[F˙n(q)ϑy (p) + F (p)ϑ˙yn(q)]
  double h_y_dot = -v_d * v_xn + Kp * (F_n * v_y + F * v_yn);

  // Common VFO
  double theta_a = atan2c(u2d * h_y, u2d * h_x);
  last_Phi = theta_a;
  double theta_a_dot = (h_y_dot * h_x - h_y * h_x_dot) / (pow(h_x, 2) + pow(h_y, 2));
  u1 = Ka * (theta_a - theta) + theta_a_dot;
  // End Common VFO
}

void vfo::VFO_point(double x_d, double y_d, double theta_d)
{

  if (e_x0 == 0.0 && e_y0 == 0.0)
  {

    e_x0 = x_d - x;
    e_y0 = y_d - y;
    e_x0_d = e_x0 * cos(theta_d) + e_y0 * sin(theta_d);
  }

  // Common VFO
  double e_x = x_d - x;
  double e_y = y_d - y;
  double e_mod = sqrt(pow(e_x, 2) + pow(e_y, 2));
  // End Common VFO

  double n = 0.8;
  double v_x = -1 * n * sgn(e_x0_d) * e_mod * cos(theta_d);
  double v_y = -1 * n * sgn(e_x0_d) * e_mod * sin(theta_d);

  // Common VFO
  double h_x = (Kp * e_x) + v_x;
  double h_y = (Kp * e_y) + v_y;
  u2 = h_x * cos(theta) + h_y * sin(theta);
  // End Common VFO

  double eq = e_x * u2 * cos(theta) + e_y * u2 * sin(theta);
  double h_x_dot = (-1 * Kp * u2 * cos(theta)) + (n * sgn(e_x0_d) * eq / e_mod * cos(theta_d));
  double h_y_dot = (-1 * Kp * u2 * sin(theta)) + (n * sgn(e_x0_d) * eq / e_mod * sin(theta_d));

  double theta_a;
  double theta_a_dot;

  if (e_mod < 0.1)
  {
    theta_a = theta_d;
    theta_a_dot = 0;
    u2 = 0;
  }
  else
  {
    // Common VFO
    theta_a = atan2c(sgn(e_x0_d) * h_y, sgn(e_x0_d) * h_x);
    theta_a_dot = (h_y_dot * h_x - h_y * h_x_dot) / (pow(h_x, 2) + pow(h_y, 2));
    // End Common VFO
  }
  last_Phi = theta_a;

  // Common VFO
  u1 = Ka * (theta_a - theta) + theta_a_dot;
  // End Common VFO

  RCLCPP_INFO(this->get_logger(), "Position: (%.3f,%.3f:%.3f) | e: %.3f,%.3f |  e_mod: %.3f, |  h: %.3f,%.3f, |  v: %.3f,%.3f, |  theta_a: %.3f, | theta_a_dot: %.3f, | u: %.3f,%.3f", x, y, theta, e_x, e_y, e_mod, h_x, h_y, v_x, v_y, theta_a, theta_a_dot, u1, u2);
}

void vfo::get_function()
{

  // Elipsa
  // F = ((pow(x, 2) / pow(1, 2)) + (pow(y, 2) / pow(2, 2)) - 1);
  // Fx = 2 * x;
  // Fxx = 2;
  // Fy = y / 2;
  // Fyy = 0.5;
  // Fxy = 0;
  // Okrąg
  F = pow((x-5),2) +pow((y-5),2) -3;
  Fx = 2*x -25;
  Fxx = 2;
  Fy = 2*y -25;
  Fyy = 2;
  Fxy = 0;
  // Sinus
  // F = sin(x) -y;
  // Fx = cos(x);
  // Fxx = -sin(x);
  // Fy = -1;
  // Fyy = 0;
  // Fxy = 0;
  // Prosta
  // F = x + 5 - y;
  // Fx = 1.0;
  // Fxx = 0.0;
  // Fy = -1.0;
  // Fyy = 0.0;
  // Fxy = 0.0;
  F *= sigma;
  RCLCPP_INFO(this->get_logger(), "X,Y: %f, %f, F: %f, Fx: %f, Fy: %f, Fxx: %f, Fyy: %f, Fxy: %f", x, y, F, Fx, Fy, Fxx, Fyy, Fxy);
}

void vfo::BSP20()
{
  u1 = u1 / (std::max(abs(u1) / max_speed, 1.0));
  u2 = u2 / (std::max(abs(u2) / max_speed, 1.0));
}

double vfo::atan2c(double y, double x)
{
  double current_atan2 = atan2(y, x);
  double last_atan2 = atan2(sin(last_Phi), cos(last_Phi));
  double big_phi = current_atan2 - last_atan2;
  double small_phi = 0;
  if (big_phi > M_PI)
  {
    small_phi = big_phi - 2 * M_PI;
  }
  else if (big_phi < -M_PI)
  {
    small_phi = big_phi + 2 * M_PI;
  }
  else
  {
    small_phi = big_phi;
  }
  return last_Phi + small_phi;
}

void vfo::publish_cmd_vel()
{
  auto cmd_vel_msg = geometry_msgs::msg::Twist();
  cmd_vel_msg.linear.x = u2;
  cmd_vel_msg.angular.z = u1;
  cmd_vel_pub->publish(cmd_vel_msg);
}

double vfo::sgn(double v)
{
  if (v < 0)
    return -1.0;
  if (v > 0)
    return 1.0;
  return 0.0;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vfo>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
