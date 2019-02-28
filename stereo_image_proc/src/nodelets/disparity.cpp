/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
//#include <boost/version.hpp>
//#if ((BOOST_VERSION / 100) % 1000) >= 53
//#include <boost/thread/lock_guard.hpp>
//#endif
//
//#include <ros/ros.h>
//#include <nodelet/nodelet.h>
//#include <image_transport/image_transport.h>
//#include <image_transport/subscriber_filter.h>
//#include <message_filters/subscriber.h>
//#include <message_filters/synchronizer.h>
//#include <message_filters/sync_policies/exact_time.h>
//#include <message_filters/sync_policies/approximate_time.h>
//
//#include <image_geometry/stereo_camera_model.h>
//#include <opencv2/calib3d/calib3d.hpp>
//#include <cv_bridge/cv_bridge.h>
//
//#include <sensor_msgs/image_encodings.h>
//#include <stereo_msgs/DisparityImage.h>
//
//#include <stereo_image_proc/DisparityConfig.h>
//#include <dynamic_reconfigure/server.h>
//
//#include <stereo_image_proc/processor.h>
#include "stereo_image_proc/disparity.hpp"

namespace stereo_image_proc {

DisparityNode::DisparityNode()
: Node("DisparityNode")
{
  onInit();

}

void DisparityNode::onInit()
{
  //ros::NodeHandle &nh = getNodeHandle();
  //ros::NodeHandle &private_nh = getPrivateNodeHandle();

  //it_.reset(new image_transport::ImageTransport(nh));

  // Synchronize inputs. Topic subscriptions happen on demand in the connection
  // callback. Optionally do approximate synchronization.
  int queue_size;
  //private_nh.param("queue_size", queue_size, 5);
  this->get_parameter_or("queue_size", queue_size, 5);
  bool approx;
  //private_nh.param("approximate_sync", approx, false);
  this->get_parameter_or("approximate_sync", approx, false);
  if (approx)
  {
    // approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(queue_size),
    //                                             sub_l_image_, sub_l_info_,
    //                                             sub_r_image_, sub_r_info_));
    // approximate_sync_->registerCallback(boost::bind(&DisparityNode::imageCb,
    //                                                this, _1, _2, _3, _4));
    approximate_sync_ = std::make_shared<ApproximateSync>(
      ApproximatePolicy(queue_size),
      sub_l_image_,
      sub_l_info_,
      sub_r_image_,
      sub_r_info_);
    approximate_sync_->registerCallback(std::bind(&DisparityNode::imageCb, this, _1, _2, _3, _4));
  }
  else
  {
    //exact_sync_.reset( new ExactSync(ExactPolicy(queue_size),
    //                                 sub_l_image_, sub_l_info_,
    //                                 sub_r_image_, sub_r_info_) );
    exact_sync_ = std::make_shared<ExactSync>(ExactPolicy(queue_size),
      sub_l_image_, sub_l_info_, sub_r_image_, sub_r_info_);
    exact_sync_->registerCallback(std::bind(&DisparityNode::imageCb, this, _1, _2, _3, _4));
  }

  // TODO:
  // Set up dynamic reconfiguration
  // ReconfigureServer::CallbackType f = boost::bind(&DisparityNode::configCb,
  //                                                this, _1, _2);
  // reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
  // reconfigure_server_->setCallback(f);


  // Monitor whether anyone is subscribed to the output
  // TODO(ros2) Implement when SubscriberStatusCallback is available
  // ros::SubscriberStatusCallback connect_cb = boost::bind(&DisparityNode::connectCb, this);
  connectCb();
  // Make sure we don't enter connectCb() between advertising and assigning to pub_disparity_
  std::lock_guard<std::mutex> lock(connect_mutex_);
  // TODO(ros2) Implement when SubscriberStatusCallback is available
  // pub_disparity_ =
  //   nh.advertise<DisparityImage>("disparity", 1, connect_cb, connect_cb);
  pub_disparity_ = create_publisher<stereo_msgs::msg::DisparityImage>("disparity");
}

// Handles (un)subscribing when clients (un)subscribe
void DisparityNode::connectCb()
{
//  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  std::lock_guard<std::mutex> lock(connect_mutex_);
  // TODO(ros2) Implement getNumSubscribers when rcl/rmw support it
  // if (pub_point_cloud_->getNumSubscribers() == 0)
  if (0) {
//  if (pub_disparity_.getNumSubscribers() == 0)
  {
    sub_l_image_.unsubscribe();
    sub_l_info_ .unsubscribe();
    sub_r_image_.unsubscribe();
    sub_r_info_ .unsubscribe();
  }
  else if (!sub_l_image_.getSubscriber())
  {
    //ros::NodeHandle &nh = getNodeHandle();
    // Queue size 1 should be OK; the one that matters is the synchronizer queue size.
    /// @todo Allow remapping left, right?
    // image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    image_transport::TransportHints hints(this, "raw");
    sub_l_image_.subscribe(this, "left/image_rect", hints.getTransport());
    //sub_l_image_.subscribe(*it_, "left/image_rect", 1, hints);
    sub_l_info_.subscribe(this, "left/camera_info");
    sub_r_image_.subscribe(this, "right/image_rect", hints.getTransport());
    sub_r_info_.subscribe(this, "right/camera_info");
  }
}

void DisparityNode::imageCb(
  const Image::ConstSharedPtr & l_image_msg,
  const CameraInfo::ConstSharedPtr & l_info_msg,
  const Image::ConstSharedPtr & r_image_msg,
  const CameraInfo::ConstSharedPtr & r_info_msg)
{
  // Update the camera model
  model_.fromCameraInfo(l_info_msg, r_info_msg);

  // Allocate new disparity image message
  DisparityImagePtr disp_msg = std::make_shared<DisparityImage>();
  disp_msg->header         = l_info_msg->header;
  disp_msg->image.header   = l_info_msg->header;

  // Compute window of (potentially) valid disparities
  int border   = block_matcher_.getCorrelationWindowSize() / 2;
  int left   = block_matcher_.getDisparityRange() + block_matcher_.getMinDisparity() + border - 1;
  int wtf = (block_matcher_.getMinDisparity() >= 0) ? border + block_matcher_.getMinDisparity() : std::max(border, -block_matcher_.getMinDisparity());
  int right  = disp_msg->image.width - 1 - wtf;
  int top    = border;
  int bottom = disp_msg->image.height - 1 - border;
  disp_msg->valid_window.x_offset = left;
  disp_msg->valid_window.y_offset = top;
  disp_msg->valid_window.width    = right - left;
  disp_msg->valid_window.height   = bottom - top;

  // Create cv::Mat views onto all buffers
  const cv::Mat_<uint8_t> l_image = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8)->image;
  const cv::Mat_<uint8_t> r_image = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8)->image;

  // Perform block matching to find the disparities
  block_matcher_.processDisparity(l_image, r_image, model_, *disp_msg);

  // Adjust for any x-offset between the principal points: d' = d - (cx_l - cx_r)
  double cx_l = model_.left().cx();
  double cx_r = model_.right().cx();
  if (cx_l != cx_r) {
    cv::Mat_<float> disp_image(disp_msg->image.height, disp_msg->image.width,
                              reinterpret_cast<float*>(&disp_msg->image.data[0]),
                              disp_msg->image.step);
    cv::subtract(disp_image, cv::Scalar(cx_l - cx_r), disp_image);
  }

  pub_disparity_.publish(disp_msg);
}


//void DisparityNode::configCb()
//{
//  // Tweak all settings to be valid
//  config.prefilter_size |= 0x1; // must be odd
//  config.correlation_window_size |= 0x1; // must be odd
//  config.disparity_range = (config.disparity_range / 16) * 16; // must be multiple of 16
//  
//  // check stereo method
//  // Note: With single-threaded NodeHandle, configCb and imageCb can't be called
//  // concurrently, so this is thread-safe.
//  block_matcher_.setPreFilterCap(config.prefilter_cap);
//  block_matcher_.setCorrelationWindowSize(config.correlation_window_size);
//  block_matcher_.setMinDisparity(config.min_disparity);
//  block_matcher_.setDisparityRange(config.disparity_range);
//  block_matcher_.setUniquenessRatio(config.uniqueness_ratio);
//  block_matcher_.setSpeckleSize(config.speckle_size);
//  block_matcher_.setSpeckleRange(config.speckle_range);
//  if (config.stereo_algorithm == stereo_image_proc::Disparity_StereoBM) { // StereoBM
//    block_matcher_.setStereoType(StereoProcessor::BM);
//    block_matcher_.setPreFilterSize(config.prefilter_size);
//    block_matcher_.setTextureThreshold(config.texture_threshold);
//  }
//  else if (config.stereo_algorithm == stereo_image_proc::Disparity_StereoSGBM) { // StereoSGBM
//    block_matcher_.setStereoType(StereoProcessor::SGBM);
//    block_matcher_.setSgbmMode(config.fullDP);
//    block_matcher_.setP1(config.P1);
//    block_matcher_.setP2(config.P2);
//    block_matcher_.setDisp12MaxDiff(config.disp12MaxDiff);
//  }
//}

}  // namespace stereo_image_proc

// Register the component with class_loader.
#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(stereo_image_proc::DisparityNode, rclcpp::Node)
