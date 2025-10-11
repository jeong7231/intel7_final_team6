#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace turtlebot3_dumpbox
{
constexpr uint16_t kDumpboxServoAddr = 360;

class ServoControllerNode : public rclcpp::Node
{
public:
  ServoControllerNode()
  : Node("dumpbox_servo_controller")
  {
    device_port_ = this->declare_parameter<std::string>("opencr_port", "/dev/ttyACM0");
    device_id_ = static_cast<uint8_t>(this->declare_parameter<int>("opencr_id", 200));
    baud_rate_ = this->declare_parameter<int>("opencr_baud_rate", 1000000);
    protocol_version_ = static_cast<float>(
      this->declare_parameter<double>("opencr_protocol_version", 2.0));
    pwm_neutral_ = this->declare_parameter<int>("servo_pwm_neutral", 1500);

    port_handler_.reset(dynamixel::PortHandler::getPortHandler(device_port_.c_str()));
    packet_handler_.reset(dynamixel::PacketHandler::getPacketHandler(protocol_version_));

    if (open_port()) {
      RCLCPP_INFO(this->get_logger(), "Connected to OpenCR (%s)", device_port_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to communicate with OpenCR on %s",
        device_port_.c_str());
    }

    pwm_command_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "turtlebot3_dumpbox/servo/pwm",
      rclcpp::QoS(10),
      std::bind(&ServoControllerNode::on_pwm_command, this, std::placeholders::_1));

    execute_service_ = this->create_service<std_srvs::srv::Trigger>(
      "turtlebot3_dumpbox/servo/apply_default",
      std::bind(&ServoControllerNode::on_apply_default, this,
      std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "ServoControllerNode ready");
  }

  ~ServoControllerNode() override
  {
    (void)write_servo_command(false);
    if (port_handler_) {
      port_handler_->closePort();
      mark_port_closed();
    }
    mark_port_closed();
  }

private:
  void mark_port_closed()
  {
    port_open_ = false;
  }
  bool open_port()
  {
    if (!port_handler_) {
      return false;
    }

    if (!port_handler_->openPort()) {
      mark_port_closed();
      return false;
    }

    if (!port_handler_->setBaudRate(baud_rate_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set baud rate %d", baud_rate_);
      port_handler_->closePort();
      mark_port_closed();
      return false;
    }

    port_open_ = true;
    return true;
  }

  bool write_servo_command(bool open)
  {
    if (!port_handler_ || !packet_handler_) {
      RCLCPP_ERROR(this->get_logger(), "Dynamixel handlers not initialised");
      return false;
    }

    if (!port_open_ && !open_port()) {
      return false;
    }

    uint8_t value = open ? 1 : 0;
    uint8_t dxl_error = 0;

    int32_t result = packet_handler_->write1ByteTxRx(
      port_handler_.get(), device_id_, kDumpboxServoAddr, value, &dxl_error);

    if (result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "DXL write failed: %s",
        packet_handler_->getTxRxResult(result));
      mark_port_closed();
      return false;
    }

    if (dxl_error != 0) {
      RCLCPP_ERROR(this->get_logger(), "DXL error: %s",
        packet_handler_->getRxPacketError(dxl_error));
      mark_port_closed();
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Servo command -> %s",
      open ? "OPEN" : "CLOSE");
    return true;
  }

  void on_pwm_command(const std_msgs::msg::Float32::SharedPtr msg)
  {
    const int pwm_value = static_cast<int>(msg->data);
    const bool open_request = pwm_value <= pwm_neutral_;
    write_servo_command(open_request);
  }

  void on_apply_default(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> & /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    bool ok = write_servo_command(false);
    response->success = ok;
    response->message = ok ? "Servo closed" : "Failed to command servo";
  }

  std::string device_port_;
  uint8_t device_id_;
  int baud_rate_;
  float protocol_version_;
  int pwm_neutral_;

  std::unique_ptr<dynamixel::PortHandler> port_handler_;
  std::unique_ptr<dynamixel::PacketHandler> packet_handler_;
  bool port_open_ {false};

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
