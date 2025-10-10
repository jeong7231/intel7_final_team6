#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace turtlebot3_dumpbox
{
class ServoControllerNode : public rclcpp::Node
{
public:
  ServoControllerNode()
  : Node("dumpbox_servo_controller"), fd_(-1)
  {
    port_ = this->declare_parameter<std::string>("opencr_port", "/dev/ttyACM0");
    baud_rate_ = this->declare_parameter<int>("opencr_baud_rate", 115200);
    pwm_min_ = this->declare_parameter<int>("servo_pwm_min", 500);
    pwm_max_ = this->declare_parameter<int>("servo_pwm_max", 2500);
    pwm_neutral_ = this->declare_parameter<int>("servo_pwm_neutral", 1500);

    pwm_command_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "turtlebot3_dumpbox/servo/pwm",
      rclcpp::QoS(10),
      std::bind(&ServoControllerNode::on_pwm_command, this, std::placeholders::_1));

    execute_service_ = this->create_service<std_srvs::srv::Trigger>(
      "turtlebot3_dumpbox/servo/apply_default",
      std::bind(&ServoControllerNode::on_apply_default, this, std::placeholders::_1,
      std::placeholders::_2));

    reconnect_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&ServoControllerNode::ensure_connection, this));

    ensure_connection();

    RCLCPP_INFO(this->get_logger(), "ServoControllerNode ready (serial commands to OpenCR)");
  }

  ~ServoControllerNode() override
  {
    send_close();
    close_serial();
  }

private:
  void ensure_connection()
  {
    if (fd_ >= 0) {
      return;
    }

    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_WARN(this->get_logger(), "Failed to open %s: %s", port_.c_str(), std::strerror(errno));
      return;
    }

    if (!configure_serial(fd_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to configure %s", port_.c_str());
      close_serial();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Connected to OpenCR on %s", port_.c_str());
    send_close();
  }

  bool configure_serial(int fd)
  {
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      return false;
    }

    cfmakeraw(&tty);

    speed_t speed = B115200;
    switch (baud_rate_) {
      case 9600: speed = B9600; break;
      case 19200: speed = B19200; break;
      case 57600: speed = B57600; break;
      case 115200: speed = B115200; break;
      default:
        RCLCPP_WARN(this->get_logger(), "Unsupported baud %d, defaulting to 115200", baud_rate_);
        speed = B115200;
        break;
    }

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr failed: %s", std::strerror(errno));
      return false;
    }

    tcflush(fd, TCIOFLUSH);
    return true;
  }

  void close_serial()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  void send_command(const std::string & command)
  {
    if (fd_ < 0) {
      ensure_connection();
    }

    if (fd_ < 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Serial port %s not open, dropping command", port_.c_str());
      return;
    }

    ssize_t written = ::write(fd_, command.c_str(), command.size());
    if (written < 0) {
      RCLCPP_ERROR(this->get_logger(), "Serial write failed: %s", std::strerror(errno));
      close_serial();
    }
  }

  void send_open()
  {
    send_command("BOX OPEN\n");
  }

  void send_close()
  {
    send_command("BOX CLOSE\n");
  }

  void on_pwm_command(const std_msgs::msg::Float32::SharedPtr msg)
  {
    int pwm_value = static_cast<int>(msg->data);
    pwm_value = std::clamp(pwm_value, pwm_min_, pwm_max_);

    const bool open_request = pwm_value <= pwm_neutral_;

    RCLCPP_INFO(
      this->get_logger(), "Received PWM %d -> command %s",
      pwm_value, open_request ? "OPEN" : "CLOSE");

    if (open_request) {
      send_open();
    } else {
      send_close();
    }
  }

  void on_apply_default(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> & /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Applying neutral (CLOSE) command");
    send_close();
    response->success = true;
    response->message = "Neutral (close) command issued";
  }

  std::string port_;
  int baud_rate_;
  int pwm_min_;
  int pwm_max_;
  int pwm_neutral_;
  int fd_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;

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
