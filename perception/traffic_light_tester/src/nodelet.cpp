// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "traffic_light_tester/nodelet.hpp"

#include <iostream>
#include <memory>
#include <utility>
#include <vector>
#include <thread>
#include <chrono>
#include <yaml-cpp/yaml.h>

/**
 * This is a helper thing to test out how well the traffic light cnn detector
 * works on real-world data.  
 * 
 * On each timer callback (I didn't want to block the main thread in the ctor in case that causes issues), do the following:
 * 
 * 1. Find the tick number (eg, 1)
 * 2. Read the corresponding ROIs from the 1_rois.yaml file
 * 3. Read the corresponding image from the 1_image.png file
 * 4. Publish rois message to topic
 * 5. Publish image to topic
 * 6. Dump anything received from the subscriber for the traffic light state
 * 7. If there are no corresponding files, just print a "done message" so the user can kill the node
 * 
*/

namespace traffic_light
{
TrafficLightTesterNodelet::TrafficLightTesterNodelet(const rclcpp::NodeOptions & options)
: Node("traffic_light_tester_node", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  // using namespace std::chrono_literals;


  timer_callback_count = 0;

  RCLCPP_INFO(this->get_logger(), "Create traffic light classifier node");
  is_approximate_sync_ = this->declare_parameter("approximate_sync", false);
  if (is_approximate_sync_) {
    approximate_sync_.reset(new ApproximateSync(ApproximateSyncPolicy(10), image_sub_, roi_sub_));
    approximate_sync_->registerCallback(
      std::bind(&TrafficLightTesterNodelet::imageRoiCallback, this, _1, _2));
  } else {
    sync_.reset(new Sync(SyncPolicy(10), image_sub_, roi_sub_));
    sync_->registerCallback(
      std::bind(&TrafficLightTesterNodelet::imageRoiCallback, this, _1, _2));
  }

  /*traffic_signal_array_pub_ =
    this->create_publisher<autoware_auto_perception_msgs::msg::TrafficSignalArray>(
      "~/output/traffic_signals", rclcpp::QoS{1}); */

  auto qos = rclcpp::QoS{1}.reliable();

  traffic_signal_array_pub_ =
    this->create_publisher<autoware_auto_perception_msgs::msg::TrafficSignalArray>(
      "~/output/traffic_signals", qos);

  roi_array_pub_ = this->create_publisher<autoware_auto_perception_msgs::msg::TrafficLightRoiArray>(
      "/traffic_light_classifier/rois", qos);

  // TODO: how to specify qos?  maybe doesn't matter, looks like it's reliable by default
  test_image_pub_ = image_transport::create_publisher(this, "/traffic_light_classifier/image_raw");  

  traffic_signal_array_sub_ = this->create_subscription<autoware_auto_perception_msgs::msg::TrafficSignalArray>(
      "/traffic_light_classifier/traffic_signals", 
      qos, 
      std::bind(&TrafficLightTesterNodelet::onTrafficSignalArray, this, _1)
  );

  using std::chrono_literals::operator""ms;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&TrafficLightTesterNodelet::timerCallback, this));

  int classifier_type = this->declare_parameter(
    "classifier_type", static_cast<int>(TrafficLightTesterNodelet::ClassifierType::HSVFilter));
  if (classifier_type == TrafficLightTesterNodelet::ClassifierType::HSVFilter) {
    RCLCPP_INFO(this->get_logger(), "Create basic color classifier");
  } else if (classifier_type == TrafficLightTesterNodelet::ClassifierType::CNN) {
#if ENABLE_GPU
    RCLCPP_INFO(this->get_logger(), "Create cnn classifier");
#else
    RCLCPP_ERROR(
      this->get_logger(), "please install CUDA, CUDNN and TensorRT to use cnn classifier");
#endif
  }
}

void TrafficLightTesterNodelet::onTrafficSignalArray(const autoware_auto_perception_msgs::msg::TrafficSignalArray::ConstSharedPtr msg)
{
  if (msg) {
    auto frame_id = msg.get()->header.frame_id;
    RCLCPP_INFO(this->get_logger(), "Msg received with frame id: %s", frame_id.c_str());
    // Read ROIs from yaml file
    bool loaded_yaml_file = false;
    YAML::Node metadata;

    std::unique_ptr<char[]> yaml_filename(new char[200]);
    snprintf(yaml_filename.get(), 200, "/home/tleyden/Development/autoware/src/universe/autoware.universe/perception/traffic_light_tester/data/%s_metadata.yaml", frame_id.c_str());
    
    try {
      metadata = YAML::LoadFile(yaml_filename.get());
      loaded_yaml_file = true;
    } catch(YAML::BadFile & e) {
      RCLCPP_INFO(this->get_logger(), "Could not load yaml file: %s", yaml_filename.get());
      loaded_yaml_file = false;
    }

    if (!loaded_yaml_file) {
      return;
    }

    // Now we know the expected label 
    auto expected_signal_color_code = metadata["expected_signal_color_code"].as<int>();
    if (msg.get()->signals.size() > 1) {
      RCLCPP_ERROR(this->get_logger(), "Expected 1 signal, got: %ld", msg.get()->signals.size());
      return;
    }

    auto traffic_signal = msg.get()->signals[0];
    if (traffic_signal.lights.size() > 1) {
      RCLCPP_ERROR(this->get_logger(), "Expected 1 light, got: %ld", traffic_signal.lights.size());
      return;
    }

    auto traffic_signal_light = traffic_signal.lights[0];
    if (traffic_signal_light.color == expected_signal_color_code) {
      RCLCPP_INFO(this->get_logger(), "Result==Success!  Frame id: %s got expected color code: %d", frame_id.c_str(), expected_signal_color_code);
    } else {
      RCLCPP_INFO(this->get_logger(), "Result==Failure.  Frame id: %s did not get expected color code: %d, got: %d", frame_id.c_str(), expected_signal_color_code, traffic_signal_light.color);
    }

  }
}

void TrafficLightTesterNodelet::timerCallback()
{

  RCLCPP_INFO(this->get_logger(), "timer_callback_count: %d", timer_callback_count);
  timer_callback_count += 1;

  // Publish each image roi pair 25 times before moving onto the next one.  This gives plenty
  // of time for the subscriber to keep up.
  auto frame_counter = (timer_callback_count / 25) + 1;
  RCLCPP_INFO(this->get_logger(), "frame_counter: %d", frame_counter);


  // Load image file
  std::unique_ptr<char[]> img_filename(new char[200]);
  snprintf(img_filename.get(), 200, "/home/tleyden/Development/autoware/src/universe/autoware.universe/perception/traffic_light_tester/data/%d.jpg", frame_counter);

  cv::Mat image = cv::imread(img_filename.get());
  if (image.empty()) {
    RCLCPP_INFO(this->get_logger(), "Failed to load image: %s", img_filename.get());
    return;
  }
  
  // Publish image
  rclcpp::Time time = rclcpp::Clock().now();
  std_msgs::msg::Header header{};
  header.stamp = time;
  std::unique_ptr<char[]> frame_id(new char[100]);
  snprintf(frame_id.get(), 100, "%d", frame_counter);
  header.frame_id = frame_id.get();
  cv_bridge::CvImage cv_img{header, sensor_msgs::image_encodings::BGR8, image};
  test_image_pub_.publish(cv_img.toImageMsg());

  // Read ROIs from yaml file
  bool loaded_yaml_file = false;
  YAML::Node metadata;
  std::unique_ptr<char[]> yaml_filename(new char[200]);
  snprintf(yaml_filename.get(), 200, "/home/tleyden/Development/autoware/src/universe/autoware.universe/perception/traffic_light_tester/data/%d_metadata.yaml", frame_counter);
  
  try {
    RCLCPP_INFO(this->get_logger(), "Read ROIs from yaml file");
    metadata = YAML::LoadFile(yaml_filename.get());
    loaded_yaml_file = true;
  } catch(YAML::BadFile & e) {
    RCLCPP_INFO(this->get_logger(), "Could not load yaml file: %s", yaml_filename.get());
    loaded_yaml_file = false;
  }

  if (!loaded_yaml_file) {
    return;
  }

  // Create and publish ROIS
  std_msgs::msg::Header header_roi{};
  header_roi.stamp = time;
  header_roi.frame_id = frame_id.get();
  autoware_auto_perception_msgs::msg::TrafficLightRoiArray rois_array{};
  sensor_msgs::msg::RegionOfInterest roi;
  roi.x_offset = metadata["x_offset"].as<int>();
  roi.y_offset = metadata["y_offset"].as<int>();
  roi.height = metadata["height"].as<int>();
  roi.width = metadata["width"].as<int>();
  RCLCPP_INFO(this->get_logger(), "Publish frame id: %s, color code: %d, x_offset: %d, y_offset: %d", frame_id.get(), metadata["expected_signal_color_code"].as<int>(), roi.x_offset, roi.y_offset);
  autoware_auto_perception_msgs::msg::TrafficLightRoi traffic_light_roi;
  traffic_light_roi.roi = roi;
  rois_array.rois.push_back(traffic_light_roi);
  rois_array.header = header_roi;
  roi_array_pub_->publish(rois_array);

}

void TrafficLightTesterNodelet::imageRoiCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
  const autoware_auto_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & input_rois_msg)
{
  RCLCPP_INFO(this->get_logger(), "imageRoiCallback invoked");
  if (classifier_ptr_.use_count() == 0) {
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(input_image_msg, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Could not convert from '%s' to 'rgb8'.",
      input_image_msg->encoding.c_str());
  }

  autoware_auto_perception_msgs::msg::TrafficSignalArray output_msg;

  for (size_t i = 0; i < input_rois_msg->rois.size(); ++i) {
    const sensor_msgs::msg::RegionOfInterest & roi = input_rois_msg->rois.at(i).roi;
    cv::Mat clipped_image(
      cv_ptr->image, cv::Rect(roi.x_offset, roi.y_offset, roi.width, roi.height));

    autoware_auto_perception_msgs::msg::TrafficSignal traffic_signal;
    traffic_signal.map_primitive_id = input_rois_msg->rois.at(i).id;
    if (!classifier_ptr_->getTrafficSignal(clipped_image, traffic_signal)) {
      RCLCPP_ERROR(this->get_logger(), "failed classify image, abort callback");
      return;
    }
    output_msg.signals.push_back(traffic_signal);
  }

  output_msg.header = input_image_msg->header;
  traffic_signal_array_pub_->publish(output_msg);
}

}  // namespace traffic_light

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(traffic_light::TrafficLightTesterNodelet)
