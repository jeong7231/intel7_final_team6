#include "turtlebot3_aruco/aruco_pose_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions{};
  auto node = std::make_shared<turtlebot3_aruco::ArucoPoseNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
