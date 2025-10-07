#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace turtlebot3_dumpbox
{
class MissionExecutorNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  MissionExecutorNode()
  : Node("dumpbox_mission_executor"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    this->declare_parameter<std::string>("target_frame", "ar_marker_target");
    this->declare_parameter<std::string>("servo_service", "turtlebot3_dumpbox/servo/apply_default");

    nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(
      this,
      "navigate_to_pose");

    servo_client_ = this->create_client<std_srvs::srv::Trigger>(
      this->get_parameter("servo_service").as_string());

    mission_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MissionExecutorNode::on_timer, this));

    RCLCPP_INFO(this->get_logger(), "MissionExecutorNode initialized (stub)");
  }

private:
  void on_timer()
  {
    mission_timer_->cancel();
    if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_WARN(this->get_logger(), "NavigateToPose action server not available");
      return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose.position.x = 0.0;
    goal_msg.pose.pose.position.y = 0.0;
    goal_msg.pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(this->get_logger(), "[stub] Sending navigation goal");

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
      std::bind(&MissionExecutorNode::on_nav_result, this, std::placeholders::_1);

    nav_action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void on_nav_result(const GoalHandleNavigate::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "[stub] Navigation succeeded, calling servo service");
      call_servo_service();
    } else {
      RCLCPP_WARN(this->get_logger(), "[stub] Navigation failed or canceled");
    }
  }

  void call_servo_service()
  {
    if (!servo_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Servo service not available");
      return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = servo_client_->async_send_request(request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future_response) {
          auto response = future_response.get();
          RCLCPP_INFO(this->get_logger(), "Servo response: %s", response->message.c_str());
        });
    (void)future;
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_client_;
  rclcpp::TimerBase::SharedPtr mission_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};
}  // namespace turtlebot3_dumpbox

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<turtlebot3_dumpbox::MissionExecutorNode>());
  rclcpp::shutdown();
  return 0;
}
