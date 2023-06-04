#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

class vfo : public rclcpp::Node
{
private:
  double last_Phi;
  double last_x, last_y;
  double sigma;
  double Kp;
  double Ka;
  double ksid;
  double v_d;
  double max_speed;
  double u2d;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

public:
  vfo();
  double atan2c(double y, double x);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void publish_cmd_vel(double v, double w);
  void calculate_velocity(const double x, const double y, const double theta_d);
  std::array<double, 2> VFO(double x, double y, double theta, double F, double Fx, double Fy, double Fxx, double Fyy, double Fxy);
  double BSP20(double u);

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

  last_Phi = 0.0;
  last_x = 0.0;
  last_y = 0.0;
  u2d = 1.0*v_d;

  
  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", sensor_qos, std::bind(&vfo::odom_callback, this, std::placeholders::_1));
  cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}


void vfo::calculate_velocity(const double x, const double y, double theta)
{
  double F, Fx, Fxx, Fy, Fyy, Fxy;
// Elipsa
  F = ((pow(x, 2) / pow(1, 2)) + (pow(y, 2) / pow(2, 2)) - 1);
  Fx = 2 * x ;
  Fxx = 2 ;
  Fy = y/2;
  Fyy = 0.5 ;
  Fxy = 0;
// Okrąg
  // F = pow((x-2),2) +pow((y-2),2) -3;
  // Fx = 2*x -4;
  // Fxx = 2;
  // Fy = 2*y -4;
  // Fyy = 2;
  // Fxy = 0;
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

  std::array<double, 2> u = VFO(x, y, theta, F, Fx, Fy, Fxx, Fyy, Fxy);

  publish_cmd_vel(BSP20(u[0]), BSP20(u[1]));
}

 std::array<double, 2> vfo::VFO(double x, double y, double theta, double F, double Fx, double Fy, double Fxx, double Fyy, double Fxy)
    {
         // ||∇F (p)||
        double delta_F_p = sqrt(pow(Fx, 2) + pow(Fy, 2));
    // VFO algorithm
        // Równanie 9
        double w_px = -Fx;
        double w_py = -Fy;
        double w2_px = -Fy;
        double w2_py = Fx;
        // Równanie 10
        //  H = v_d * w⊥(p) / ||∇F(p)|| + Kp * F * w(p) / ||∇F(p)||
        double h_x = v_d * w2_px / delta_F_p  + Kp * F * w_px / delta_F_p;
        double h_y = v_d * w2_py / delta_F_p  + Kp * F * w_py / delta_F_p;
        // u2 = h_x * cos(theta) + h_y * sin(theta)
        double u2 = h_x * cos(theta) + h_y * sin(theta);
        // θa(p) = Atan2c ( ksid * h_y, ksid * h_x)
        // double tetaa = atan2(ksid * h_y, ksid * h_x);
        double tetaa = atan2c(u2d*h_y, u2d*h_x);
        last_Phi = tetaa;
        // ea(q)  θa(p) − θ
        double Etetaa = tetaa - theta;
        // and η1 = FxFxy − FyFxx , η2 = FxFyy − FyFxy , η3 = FyFxx − FxFxy ,η4 = FyFxy −FxFyy . 
        double n1, n2, n3 ,n4;
        n1 = Fx * Fxy - Fy * Fxx;
        n2 = Fx * Fyy - Fy * Fxy;
        n3 = Fy * Fxx - Fx * Fxy;
        n4 = Fy * Fxy - Fx * Fyy;
        
        double v_xn = u2 / pow(delta_F_p, 3)* (Fy * (n1 * cos(theta) + n2 * sin(theta)) );
        double v_yn = u2 / pow(delta_F_p, 3)* (Fx * (n3 * cos(theta) + n4 * sin(theta)) );
        //Fn (Fx cθ + Fy sθ )(hx cθ + hy sθ )
        double F_n = (Fx * cos(theta) + Fy * sin(theta)) * (h_x * cos(theta) + h_y * sin(theta));
        double v_x = -Fx / delta_F_p; 
        double v_y = -Fy / delta_F_p;
        // vdϑ˙yn(q)+kp[F˙n(q)ϑx (p) + F (p)ϑ˙xn(q)]
        double h_xn = v_d*v_yn + Kp * (F_n  * v_x + F * v_xn);
        // −vdϑ˙xn(q)+kp[F˙n(q)ϑy (p) + F (p)ϑ˙yn(q)]
        double h_yn = -v_d*v_xn + Kp * (F_n  * v_y + F * v_yn);
        double theta_an = h_yn * h_x - h_y*h_xn;
        theta_an /= (pow(h_x, 2) + pow(h_y, 2));
        // hθ (q) = ka * ea(q) + θ˙an(q)
        double u1 = Ka * Etetaa + theta_an;
        
        last_x = h_x;
        last_y = h_y;
        return std::array<double, 2>{u1, u2};
    }

void vfo::publish_cmd_vel(double u1, double u2)
{
  auto cmd_vel_msg = geometry_msgs::msg::Twist();
  cmd_vel_msg.linear.x = u1;
  cmd_vel_msg.angular.z = u2;
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

  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  calculate_velocity(x, y, yaw);
}

double vfo::BSP20(double u)
{
    return u / (std::max(abs(u) / max_speed, 1.0));
}

double vfo::atan2c(double y, double x)
{
  double current_atan2 = atan2(y, x);
  double last_atan2 = atan2(last_y, last_x);
  double delta_phi = current_atan2 - last_atan2;
  double delta_Phi = 0;
  if (delta_phi > M_PI)
  {
    delta_Phi = delta_phi - 2 * M_PI;
  }
  else if (delta_phi < -M_PI)
  {
    delta_Phi = delta_phi +2 * M_PI;
  }
  return last_Phi + delta_Phi;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vfo>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
