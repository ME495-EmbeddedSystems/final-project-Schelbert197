#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <franka/robot.h>
// #include "franka_hardware/robot.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "force_msgs/msg/force.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("force_publisher")
    {
      // publisher_ = this->create_publisher<std_msgs::msg::String>("topic",10);
      publisher_ = this->create_publisher<force_msgs::msg::Force>("franka_ee_force", 10);
      // publisher_ = this->create_publisher<std_msgs::msg::String>("topic",10);

      this->robot_ = std::make_unique<franka::Robot>("panda0.robot");

      auto publish_msg = [this]() -> void {
        franka::RobotState robot_state = robot_->readOnce();

        std::array<double, 6> external_force_torque = robot_state.K_F_ext_hat_K;

        auto message = force_msgs::msg::Force();
        message.ee_force = external_force_torque;

        this->publisher_->publish(message);
      };
      timer_ = this->create_wall_timer(1s, publish_msg);
    }
  
  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<force_msgs::msg::Force>::SharedPtr publisher_;
    std::shared_ptr<franka::Robot> robot_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
