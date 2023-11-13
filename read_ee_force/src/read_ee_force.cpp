#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <franka/robot.h>
// #include "franka_hardware/robot.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("force_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic",10);
      // publisher_ = this->create_publisher<std_msgs::msg::String>("topic",10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
      this->robot_ = std::make_unique<franka::Robot>("panda0.robot");
      
    }
  
  private:
    void timer_callback()
    {
      franka::RobotState robot_state = robot_->readOnce();

      std::array<double, 6> external_force_torque = robot_state.O_F_ext_hat_K;

      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::shared_ptr<franka::Robot> robot_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
