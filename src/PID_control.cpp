#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>

class PIDController : public rclcpp::Node {
public:
  PIDController(double kp, double ki, double kd, double target)
      : Node("pid_controller"), kp(kp), ki(ki), kd(kd), sum_error(0),
        prev_error(0), dt(0.03), end_controller(false) {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 30);
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 20, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          this->set_pose(msg);
        });
    timer = this->create_wall_timer(std::chrono::milliseconds(30), [this]() {
      if (!end_controller) {
        this->calculate();
      }
    });
  }

  void set_pose(const nav_msgs::msg::Odometry::SharedPtr msg) {
    posisi = msg->pose.pose.position.x;
  }

  void calculate() {
    double error = abs(target - posisi);
    sum_error += error;
    prev_error = error;

    double p = kp * error;
    double i = ki * sum_error;
    double d = kd * (error - prev_error) / dt;

    double output = p + i + d;
    if (std::abs(output) > 0.8 || std::abs(output) < 0.8)
      output = 0.8 * std::abs(output);

    geometry_msgs::msg::Twist twist_msg_;
    twist_msg_.linear.x = output;

    if (error < 0.05) // toleransi jarak
    {
      RCLCPP_INFO(this->get_logger(), "Robot telah sampai tujuan");
      RCLCPP_INFO(this->get_logger(), "Posisi = %f", posisi);
      twist_msg_.linear.x = 0.0;
      sum_error = 0;
      end_controller = true;
    }

    publisher_->publish(twist_msg_);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::TimerBase::SharedPtr timer;

  double kp;
  double ki;
  double kd;
  double dt;
  double target;
  double posisi;
  double sum_error;
  double prev_error;
  bool end_controller;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto pid = std::make_shared<PIDController>(1.0, 0.1, 0.01, 3.0);

  rclcpp::spin(pid);

  rclcpp::shutdown();

  return 0;
}
