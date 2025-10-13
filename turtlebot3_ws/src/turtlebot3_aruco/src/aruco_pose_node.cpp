#include "turtlebot3_aruco/aruco_pose_node.hpp"

#include "rclcpp_components/register_node_macro.hpp"
#include <cmath>
#include <stdexcept>

#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace turtlebot3_aruco
{
namespace
{
constexpr double kWarnThrottleMs = 2000.0;

geometry_msgs::msg::TransformStamped buildTransform(
  const std_msgs::msg::Header & header,
  const std::string & child_frame,
  const tf2::Vector3 & translation,
  const tf2::Quaternion & rotation)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header = header;
  transform.child_frame_id = child_frame;
  transform.transform.translation.x = translation.x();
  transform.transform.translation.y = translation.y();
  transform.transform.translation.z = translation.z();
  transform.transform.rotation = tf2::toMsg(rotation);
  return transform;
}

std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> createDictionaryLookup()
{
  return {
    {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
    {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
    {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
    {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
    {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
    {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
    {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
    {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
    {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
    {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
    {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
    {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
    {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
    {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
    {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
    {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
    {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL}
  };
}
}  // namespace

ArucoPoseNode::ArucoPoseNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("turtlebot3_aruco_tracker", options)
{
  marker_size_ = this->declare_parameter<double>("marker_size", marker_size_);
  camera_frame_ = this->declare_parameter<std::string>("camera_frame", camera_frame_);
  image_topic_ = this->declare_parameter<std::string>("image_topic", image_topic_);
  camera_info_topic_ = this->declare_parameter<std::string>("camera_info_topic", camera_info_topic_);
  publish_debug_image_ = this->declare_parameter<bool>("publish_debug_image", publish_debug_image_);
  const auto dictionary_name = this->declare_parameter<std::string>("aruco_dictionary", "DICT_5X5_250");

  initialiseDictionary(dictionary_name);
  detector_params_ = cv::aruco::DetectorParameters::create();

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, rclcpp::QoS(10),
    std::bind(&ArucoPoseNode::cameraInfoCallback, this, std::placeholders::_1));

  image_sub_ = image_transport::create_subscription(
    this, image_topic_,
    std::bind(&ArucoPoseNode::imageCallback, this, std::placeholders::_1),
    "raw");

  if (publish_debug_image_) {
    debug_image_pub_ = image_transport::create_publisher(this, "aruco/detection_image");
  }

  RCLCPP_INFO(this->get_logger(), "ArucoPoseNode ready (dict: %s, marker_size: %.3f m)",
    dictionary_name.c_str(), marker_size_);
}

void ArucoPoseNode::initialiseDictionary(const std::string & dictionary_name)
{
  static const auto dictionary_lookup = createDictionaryLookup();
  auto it = dictionary_lookup.find(dictionary_name);
  if (it == dictionary_lookup.end()) {
    RCLCPP_WARN(this->get_logger(),
      "Unknown dictionary '%s', defaulting to DICT_5X5_250", dictionary_name.c_str());
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
    return;
  }
  dictionary_ = cv::aruco::getPredefinedDictionary(it->second);
}

void ArucoPoseNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double *>(msg->k.data())).clone();
  distortion_coeffs_ = cv::Mat(msg->d).clone();
  camera_info_received_ = true;
  RCLCPP_INFO(this->get_logger(), "Camera info received");

  if (camera_info_sub_) {
    camera_info_sub_.reset();
  }
}

void ArucoPoseNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (!camera_info_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), kWarnThrottleMs,
      "Waiting for camera info before processing images");
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
      cv_ptr = cv_bridge::toCvShare(msg);
    } else {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
  } catch (const cv_bridge::Exception & ex) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", ex.what());
    return;
  }

  cv::Mat gray;
  if (cv_ptr->image.channels() == 1) {
    gray = cv_ptr->image;
  } else {
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
  }

  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<int> marker_ids;
  cv::aruco::detectMarkers(gray, dictionary_, marker_corners, marker_ids, detector_params_);

  if (marker_ids.empty()) {
    return;
  }

  cv::Mat rvecs;
  cv::Mat tvecs;
  cv::aruco::estimatePoseSingleMarkers(
    marker_corners, marker_size_, camera_matrix_, distortion_coeffs_, rvecs, tvecs);

  tf2::Quaternion correction_quat;
  correction_quat.setRPY(-M_PI_2, 0.0, -M_PI_2);
  tf2::Matrix3x3 correction_matrix(correction_quat);

  std_msgs::msg::Header header;
  header.stamp = msg->header.stamp;
  header.frame_id = camera_frame_;

  for (size_t i = 0; i < marker_ids.size(); ++i) {
    cv::Vec3d rvec = rvecs.at<cv::Vec3d>(static_cast<int>(i));
    cv::Vec3d tvec = tvecs.at<cv::Vec3d>(static_cast<int>(i));

    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);

    tf2::Matrix3x3 marker_matrix(
      rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
      rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
      rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));

    tf2::Matrix3x3 corrected_matrix = correction_matrix * marker_matrix;

    tf2::Vector3 translation(tvec[0], tvec[1], tvec[2]);
    tf2::Vector3 corrected_translation = correction_matrix * translation;

    tf2::Quaternion corrected_quaternion;
    corrected_matrix.getRotation(corrected_quaternion);
    corrected_quaternion.normalize();

    auto transform = buildTransform(
      header,
      "ar_marker_" + std::to_string(marker_ids[i]),
      corrected_translation,
      corrected_quaternion);

    broadcastMarkerTransform(transform);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), kWarnThrottleMs,
      "Tracked marker %d", marker_ids[i]);
  }

  if (publish_debug_image_ && debug_image_pub_.getNumSubscribers() > 0) {
    cv::Mat debug_image = cv_ptr->image.clone();
    cv::aruco::drawDetectedMarkers(debug_image, marker_corners, marker_ids);
    for (size_t i = 0; i < marker_ids.size(); ++i) {
      cv::Vec3d rvec = rvecs.at<cv::Vec3d>(static_cast<int>(i));
      cv::Vec3d tvec = tvecs.at<cv::Vec3d>(static_cast<int>(i));
      cv::aruco::drawAxis(debug_image, camera_matrix_, distortion_coeffs_, rvec, tvec, marker_size_ * 0.5);
    }

    auto debug_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, debug_image).toImageMsg();
    debug_image_pub_.publish(debug_msg);
  }
}

void ArucoPoseNode::broadcastMarkerTransform(
  const geometry_msgs::msg::TransformStamped & transform) const
{
  tf_broadcaster_->sendTransform(transform);
}

}  // namespace turtlebot3_aruco

RCLCPP_COMPONENTS_REGISTER_NODE(turtlebot3_aruco::ArucoPoseNode)

