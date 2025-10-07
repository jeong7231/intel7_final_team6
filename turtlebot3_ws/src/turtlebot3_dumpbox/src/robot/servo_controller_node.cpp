#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float32.hpp"

namespace turtlebot3_dumpbox
{
class ServoControllerNode : public rclcpp::Node
{
public:
  ServoControllerNode()
  : Node("dumpbox_servo_controller")
  {
    this->declare_parameter<std::string>("opencr_port", "/dev/ttyACM0");
    this->declare_parameter<int>("servo_pwm_min", 500);
    this->declare_parameter<int>("servo_pwm_max", 2500);
    this->declare_parameter<int>("servo_pwm_neutral", 1500);

    pwm_command_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "turtlebot3_dumpbox/servo/pwm",
      rclcpp::QoS(10),
      std::bind(&ServoControllerNode::on_pwm_command, this, std::placeholders::_1));

    execute_service_ = this->create_service<std_srvs::srv::Trigger>(
      "turtlebot3_dumpbox/servo/apply_default",
      std::bind(&ServoControllerNode::on_apply_default, this, std::placeholders::_1,
      std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "ServoControllerNode initialized (stub)");
  }

private:
  void on_pwm_command(const std_msgs::msg::Float32::SharedPtr msg)
  {
    const auto pwm_value = static_cast<int>(msg->data);
    RCLCPP_INFO(this->get_logger(), "[stub] Received PWM command: %d", pwm_value);
    // TODO: Integrate with OpenCR control table write when firmware is ready.
  }

  void on_apply_default(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> & /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    const int neutral_pwm = this->get_parameter("servo_pwm_neutral").as_int();
    RCLCPP_INFO(this->get_logger(), "[stub] Applying neutral PWM %d", neutral_pwm);
    response->success = true;
    response->message = "Neutral PWM command issued (stub)";
    // TODO: Issue neutral command to OpenCR.
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pwm_command_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_service_;
};
}  // namespace turtlebot3_dumpbox

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<turtlebot3_dumpbox::ServoControllerNode>());
  rclcpp::shutdown();
  return 0;
}
