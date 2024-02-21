#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <unistd.h>

class RobotController : public rclcpp::Node {
public:
  RobotController() : Node("robot_controller") {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    twist_msg_.linear.x = 0.0;
    twist_msg_.angular.z = 0.0;

    speed_x = 0.4;
    speed_z = 0.4;

    std::thread(std::bind(&RobotController::keyboardLoop, this)).detach();
  }

private:
  double speed_x;
  double speed_z;
  void keyboardLoop() {
    while (rclcpp::ok()) {
      int ch = getchar();

      switch (ch) {
      case 'u':
        twist_msg_.linear.x = speed_x;
        twist_msg_.angular.z = speed_z;
        break;
      case 'i':
        twist_msg_.linear.x = speed_x;
        twist_msg_.angular.z = 0;
        break;
      case 'o':
        twist_msg_.linear.x = speed_x;
        twist_msg_.angular.z = -1 * speed_z;
        break;
      case 'j':
        twist_msg_.linear.x = 0;
        twist_msg_.angular.z = speed_z;
        break;
      case 'k':
        twist_msg_.linear.x = 0.0;
        twist_msg_.angular.z = 0.0;
        break;
      case 'l':
        twist_msg_.linear.x = 0;
        twist_msg_.angular.z = -1 * speed_z;
        break;
      case 'm':
        twist_msg_.linear.x = -1 * speed_x;
        twist_msg_.angular.z = speed_z;
        break;
      case ',':
        twist_msg_.linear.x = -1 * speed_x;
        twist_msg_.angular.z = 0;
        break;
      case '.':
        twist_msg_.linear.x = -1 * speed_x;
        twist_msg_.angular.z = -1 * speed_z;
        break;
      case 'q':
        speed_x += 0.1;
        twist_msg_.linear.x = speed_x;
        break;
      case 'z':
        speed_x -= 0.1;
        twist_msg_.linear.x = speed_x;
        break;
      case 'w':
        speed_z += 0.1;
        break;
      case 'x':
        speed_z -= 0.1;
        break;
      default:
        break;
      }

      publisher_->publish(twist_msg_);
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  geometry_msgs::msg::Twist twist_msg_;
};

int main(int argc, char **argv) {

  std::cout << "Panduan tombol keyboard untuk menggerakkan robot!\n";
  std::cout << "---------------------------\n";
  std::cout << "Gerakan:\n";
  std::cout << "   u    i    o\n";
  std::cout << "   j    k    l\n";
  std::cout << "   m    ,    .\n";
  std::cout << "---------------------------\n";
  std::cout << "u : maju ke kiri\n";
  std::cout << "i : maju ke depan\n";
  std::cout << "o : maju ke kanan\n";
  std::cout << "j : putar kiri\n";
  std::cout << "k : berhenti\n";
  std::cout << "l : putar kanan\n";
  std::cout << "m : mundur ke kiri\n";
  std::cout << ", : mundur ke belakang\n";
  std::cout << ". : mundur ke kanan\n";
  std::cout << "i : maju ke depan\n";
  std::cout << "---------------------------\n";
  std::cout << "q/z : menambah/mengurangi kecepatan linier robot (0.1)\n";
  std::cout << "w/x : menambah/mengurangi kecepatan angular robot (0.05)\n";
  std::cout << "---------------------------\n";
  std::cout << "Tekan CTRL-C untuk keluar\n";
  std::cout << "---------------------------\n";
  std::cout << "(jangan lupa tekan 'Enter' setelah tekan tombol)\n";

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotController>());
  rclcpp::shutdown();
  return 0;
}
