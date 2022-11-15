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
      "/perception/traffic_light_recognition/rois", qos);

  // TODO: how to specify qos?  maybe doesn't matter, looks like it's reliable by default
  test_image_pub_ = image_transport::create_publisher(this, "/sensing/camera/traffic_light/image_raw");  

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
  RCLCPP_INFO(this->get_logger(), "onTrafficSignalArray() callback");
  if (msg) {
    RCLCPP_INFO(this->get_logger(), "Msg received");
  }
}

void TrafficLightTesterNodelet::timerCallback()
{

  RCLCPP_INFO(this->get_logger(), "timer_callback_count: %d", timer_callback_count);
  timer_callback_count += 1;

  // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // Load image file
  cv::Mat image = cv::imread("/home/tleyden/Development/autoware/src/universe/autoware.universe/perception/traffic_light_tester/data/1.jpg");
  if (image.empty()) {
    RCLCPP_INFO(this->get_logger(), "failed to load image");
  } 

  // PUblish image
  rclcpp::Time time = rclcpp::Clock().now();
  std_msgs::msg::Header header{};
  header.stamp = time;
  header.frame_id = "test_image";
  cv_bridge::CvImage cv_img{header, sensor_msgs::image_encodings::BGR8, image};
  test_image_pub_.publish(cv_img.toImageMsg());



  // Load ROIs file (or use a hardcoded hashmap)
    // const sensor_msgs::msg::RegionOfInterest & roi = input_rois_msg->rois.at(i).roi;
    // cv::Mat clipped_image(
    //   cv_ptr->image, cv::Rect(roi.x_offset, roi.y_offset, roi.width, roi.height));
    //   const autoware_auto_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & input_rois_msg



  // TODO: figure out why this isn't working.  Look at traffic_light_ssd_fine_detector for an example of publishing a TrafficLightRoiArray
  std_msgs::msg::Header header_roi{};
  header_roi.stamp = time;
  header_roi.frame_id = "test_roi";
  autoware_auto_perception_msgs::msg::TrafficLightRoiArray rois_array;
  sensor_msgs::msg::RegionOfInterest roi;
  roi.x_offset = 0; // fake
  roi.y_offset = 0; // fake
  roi.height = 10;  // fake values, just to get things working
  roi.width = 10; // fake
  autoware_auto_perception_msgs::msg::TrafficLightRoi traffic_light_roi;
  traffic_light_roi.roi = roi;
  rois_array.rois.push_back(traffic_light_roi);
  rois_array.header = header_roi;

  // auto rois_array_ptr = std::make_unique<autoware_auto_perception_msgs::msg::TrafficLightRoiArray>(rois_array);

  roi_array_pub_->publish(rois_array);



  // Call model - we would need all those cuda libraries


  // Print out results


  // Every timer callback, publish an empty traffic signal.  Just an experiment
  // to see if publishing woould work.
  autoware_auto_perception_msgs::msg::TrafficSignalArray output_msg;
  autoware_auto_perception_msgs::msg::TrafficSignal traffic_signal;
  output_msg.signals.push_back(traffic_signal);
  traffic_signal_array_pub_->publish(output_msg);


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
