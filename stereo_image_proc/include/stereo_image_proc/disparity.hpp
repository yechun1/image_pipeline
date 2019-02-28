// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#ifndef STEREO_IMAGE_PROC__DISPARITY_NODELET_HPP_
#define STEREO_IMAGE_PROC__DISPARITY_NODELET_HPP_

//#include <rclcpp/rclcpp.hpp>
//#include <image_publisher/visibility.h>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <string>

//#include <boost/version.hpp>
//#if ((BOOST_VERSION / 100) % 1000) >= 53
//#include <boost/thread/lock_guard.hpp>
//#endif

//#include <ros/ros.h>
//#include <nodelet/nodelet.h>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_geometry/stereo_camera_model.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

//#include <sensor_msgs/image_encodings.h>
//#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/image_encodings.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>

//#include "stereo_image_proc/DisparityConfig.h"
//#include <dynamic_reconfigure/server.h>

#include "stereo_image_proc/visibility.h"
#include "stereo_image_proc/processor.h"

namespace stereo_image_proc
{

using namespace sensor_msgs;
using namespace stereo_msgs;
//using namespace message_filters::sync_policies;
using namespace std::placeholders;

class DisparityNode : public rclcpp::Node
{
public:
  STEREO_IMAGE_PROC_PUBLIC DisparityNode();


private:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
//  image_transport::CameraPublisher pub_;
//
//  cv::VideoCapture cap_;
//  cv::Mat image_;
//  rclcpp::TimerBase::SharedPtr timer_;
//
//  std::string filename_;
//  bool flip_horizontal_;
//  bool flip_vertical_;
//
//  std::string frame_id_;
//  double publish_rate_;
//  std::string camera_info_url_;
//  bool flip_image_;
//  int flip_value_;
//  sensor_msgs::msg::CameraInfo camera_info_;
//
//  rclcpp::Logger logger_ = rclcpp::get_logger("DisparityNode");
//  boost::shared_ptr<image_transport::ImageTransport> it_;
  
  // Subscriptions
  image_transport::SubscriberFilter sub_l_image_, sub_r_image_;
  message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;
  typedef ExactTime<Image, CameraInfo, Image, CameraInfo> ExactPolicy;
  typedef ApproximateTime<Image, CameraInfo, Image, CameraInfo> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  std::shared_ptr<ExactSync> exact_sync_;
  std::shared_ptr<ApproximateSync> approximate_sync_;
  // Publications
//  boost::mutex connect_mutex_;
  std::mutex connect_mutex_;
//  ros::Publisher pub_disparity_;
  rclcpp::Publisher<DisparityImage>::SharedPtr pub_disparity_;

  // Dynamic reconfigure
//  boost::recursive_mutex config_mutex_;
//  typedef stereo_image_proc::DisparityConfig Config;
//  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
//  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  
  // Processing state (note: only safe because we're single-threaded!)
  image_geometry::StereoCameraModel model_;
  stereo_image_proc::StereoProcessor block_matcher_; // contains scratch buffers for block matching

  void onInit();
  void connectCb();
  void imageCb(
    const Image::ConstSharedPtr & l_image_msg,
    const CameraInfo::ConstSharedPtrPtr & l_info_msg,
    const Image::ConstSharedPtr & r_image_msg,
    const CameraInfo::ConstSharedPtr & r_info_msg);
  void configCb();
};

}  // namespace stereo_image_proc

#endif  // STEREO_IMAGE_PROC__DISPARITY_NODELET_HPP_
