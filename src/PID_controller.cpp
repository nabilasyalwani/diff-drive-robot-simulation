#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <stdio.h>

class PIDController : public rclcpp::Node {
public:
  PIDController()
      : Node("pid_controller"), first_pose(false), final_pose(false),
        end_controller(false), target_x(3.0), target_y(4.0), target_w(-1.56),
        kp(1.0), ki(0.1), sum_error(0.0) {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 30);
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 30,
        std::bind(&PIDController::movement, this, std::placeholders::_1));
    twist_msg_.angular.z = 0.0;
  }

  void movement(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double posisi_x = msg->pose.pose.position.x;
    double posisi_y = msg->pose.pose.position.y;

    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    double yaw =
        std::atan2((2 * (qw * qz + qx * qy)), (1 - 2 * (qy * qy + qz * qz)));

    if (!first_pose) {
      double target_yaw = std::atan2(target_y - posisi_y, target_x - posisi_x);
      double yaw_error = range_phi(target_yaw - yaw);

      twist_msg_.angular.z = yaw_error;
      publisher_->publish(twist_msg_);

      if (std::fabs(yaw - target_yaw) <= 0.05) { // toleransi sudut
        twist_msg_.angular.z = 0.000000000;
        first_pose = true;
        RCLCPP_INFO(this->get_logger(), "Berhasil berbelok ke arah tujuan");
        rclcpp::sleep_for(std::chrono::seconds(1));
      }
    }

    if (first_pose && !end_controller) {
      double jarak = std::sqrt(std::pow(target_y - posisi_y, 2.0) +
                               std::pow(target_x - posisi_x, 2.0));
      RCLCPP_INFO(this->get_logger(), "Jarak sampai tujuan: %lf", jarak);

      double error = jarak;
      sum_error += error;
      double p = kp * error;
      double i = ki * sum_error;

      double v = p + i;

      if (std::fabs(v) > 0.8) {
        v = 0.8;
      }
      twist_msg_.linear.x = v;
      twist_msg_.angular.z = 0.000000000;
      publisher_->publish(twist_msg_);

      if (jarak < 0.5) { // toleransi jarak
        RCLCPP_INFO(this->get_logger(), "Robot telah sampai tujuan");
        twist_msg_.linear.x = 0.000000000;
        twist_msg_.angular.z = 0.000000000;
        end_controller = true;
        sum_error = 0.0;
      }
    }

    if (first_pose && end_controller && !final_pose) {
      double yaw_error = (target_w - yaw);
      double target_yaw = range_phi(yaw_error);

      twist_msg_.angular.z = target_yaw;
      publisher_->publish(twist_msg_);

      if (std::fabs(yaw - target_w) <= 0.1) {
        twist_msg_.angular.z = 0.000000000;
        final_pose = true;
        RCLCPP_INFO(this->get_logger(),
                    "Odom: posisi x = %lf, posisi y = %lf, Yaw = %lf", posisi_x,
                    posisi_y, yaw);
      }
    }
  }

  double range_phi(double x) {
    x = fmod(x, 2 * M_PI);
    if (x > M_PI) {
      x -= 2 * M_PI;
    } else if (x < -M_PI) {
      x += 2 * M_PI;
    }
    return x;
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  geometry_msgs::msg::Twist twist_msg_;
  bool first_pose, final_pose, end_controller;
  double target_x, target_y, target_w;
  double kp, ki, kd;
  double sum_error;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto pid = std::make_shared<PIDController>();
  rclcpp::spin(pid);
  rclcpp::shutdown();
  return 0;
}
