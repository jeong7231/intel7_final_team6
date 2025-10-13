#ifndef TURTLEBOT3_ARUCO__ARUCO_POSE_NODE_HPP_
#define TURTLEBOT3_ARUCO__ARUCO_POSE_NODE_HPP_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

namespace turtlebot3_aruco
{
class ArucoPoseNode : public rclcpp::Node
{
public:
  explicit ArucoPoseNode(const rclcpp::NodeOptions & options);

private:
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void initialiseDictionary(const std::string & dictionary_name);
  void broadcastMarkerTransform(
    const geometry_msgs::msg::TransformStamped & transform) const;
  void computeRelativeDistances();

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  image_transport::Subscriber image_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  cv::Mat camera_matrix_;
  cv::Mat distortion_coeffs_;
  bool camera_info_received_ {false};

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

  double marker_size_ {0.04};
  std::string camera_frame_ {"camera_rgb_frame"};
  std::string base_frame_ {"base_link"};
  std::string image_topic_ {"/camera/image_raw"};
  std::string camera_info_topic_ {"/camera/camera_info"};
  bool publish_debug_image_ {false};

  image_transport::Publisher debug_image_pub_;
  rclcpp::TimerBase::SharedPtr distance_timer_;
  std::set<int> tracked_marker_ids_;
};
}  // namespace turtlebot3_aruco

#endif  // TURTLEBOT3_ARUCO__ARUCO_POSE_NODE_HPP_
