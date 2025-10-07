#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace turtlebot3_dumpbox
{
class HeartbeatNode : public rclcpp::Node
{
public:
  HeartbeatNode()
  : Node("dumpbox_heartbeat")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>(
      "turtlebot3_dumpbox/heartbeat", 10);
    timer_ = this->create_wall_timer(1s, std::bind(&HeartbeatNode::on_timer, this));
  }

private:
  void on_timer()
  {
    std_msgs::msg::String msg;
    msg.data = "turtlebot3_dumpbox ready";
    publisher_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace turtlebot3_dumpbox

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<turtlebot3_dumpbox::HeartbeatNode>());
  rclcpp::shutdown();
  return 0;
}
