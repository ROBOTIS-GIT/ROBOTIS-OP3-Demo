/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Kayman Jung */
#ifndef _BALL_DETECTOR_H_
#define _BALL_DETECTOR_H_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
// #include <dynamic_reconfigure/server.h>
#include "op3_ball_detector_msgs/msg/circle_set_stamped.hpp"
#include "op3_ball_detector_msgs/msg/ball_detector_params.hpp"
#include "op3_ball_detector_msgs/srv/get_parameters.hpp"
#include "op3_ball_detector_msgs/srv/set_parameters.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include "op3_ball_detector/ball_detector_config.h"
//#include "op3_ball_detector/detector_params_config.h"

namespace robotis_op
{

class BallDetector : public rclcpp::Node
{
 public:
  BallDetector();
  ~BallDetector();

  //checks if a new image has been received
  bool newImage();

  //execute circle detection with the current image
  void process();

  //publish the output image (input image + marked circles)
  void publishImage();

  //publish the circle set data
  void publishCircles();
  
  // init imageport
  void initialize();

 protected:
  const static int NOT_FOUND_TH = 30;

  //callbacks to image subscription
  void imageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg);

  //callbacks to camera info subscription
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  //void dynParamCallback(DetectorParamsConfig &config, uint32_t level);
  void enableCallback(const std_msgs::msg::Bool::SharedPtr msg);

  void paramCommandCallback(const std_msgs::msg::String::SharedPtr msg);
  bool setParamCallback(const std::shared_ptr<op3_ball_detector_msgs::srv::SetParameters::Request> req,
                        std::shared_ptr<op3_ball_detector_msgs::srv::SetParameters::Response> res);
  bool getParamCallback(const std::shared_ptr<op3_ball_detector_msgs::srv::GetParameters::Request> req,
                        std::shared_ptr<op3_ball_detector_msgs::srv::GetParameters::Response> res);
  void resetParameter();
  void publishParam();

  void printConfig();
  void saveConfig();
  void setInputImage(const cv::Mat & inIm);
  void setInputImage(const cv::Mat & inIm, cv::Mat &in_filter_img);
  void getOutputImage(cv::Mat & outIm);
  void filterImage();
  void filterImage(const cv::Mat &in_filter_img, cv::Mat &out_filter_img);
  void makeFilterMask(const cv::Mat &source_img, cv::Mat &mask_img, int range);
  void makeFilterMaskFromBall(const cv::Mat &source_img, cv::Mat &mask_img);
  void inRangeHsv(const cv::Mat &input_img, const HsvFilter &filter_value, cv::Mat &output_img);
  void mophology(const cv::Mat &intput_img, cv::Mat &output_img, int ellipse_size);
  void houghDetection(const unsigned int imgEncoding);
  void houghDetection2(const cv::Mat &input_hough);
  void drawOutputImage();

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;

  //image publisher/subscriber
  image_transport::ImageTransport* it_;
  image_transport::Publisher image_pub_;
  cv_bridge::CvImage cv_img_pub_;
  // image_transport::Subscriber image_sub_;
  cv_bridge::CvImagePtr cv_img_ptr_sub_;

  bool enable_;
  bool init_param_;
  int not_found_count_;

  //circle set publisher
  op3_ball_detector_msgs::msg::CircleSetStamped circles_msg_;
  rclcpp::Publisher<op3_ball_detector_msgs::msg::CircleSetStamped>::SharedPtr circles_pub_;

  //camera info subscriber
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  //dynamic reconfigure
  DetectorConfig params_config_;
  std::string param_path_;
  bool has_path_;

  // web setting
  std::string default_setting_path_;
  rclcpp::Publisher<op3_ball_detector_msgs::msg::BallDetectorParams>::SharedPtr param_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr param_command_sub_;
  rclcpp::Service<op3_ball_detector_msgs::srv::GetParameters>::SharedPtr get_param_client_;
  rclcpp::Service<op3_ball_detector_msgs::srv::SetParameters>::SharedPtr set_param_client_;

  //flag indicating a new image has been received
  bool new_image_flag_;

  //image time stamp and frame id
  rclcpp::Time sub_time_;
  std::string image_frame_id_;

  //img encoding id
  unsigned int img_encoding_;

  /** \brief Set of detected circles
   *
   * Detected circles. For a circle i:
   *    x_i: circles[i][0]
   *    y_i: circles[i][1]
   *    radius_i: circles[i][2]
   *
   **/
  std::vector<cv::Vec3f> circles_;
  cv::Mat in_image_;
  cv::Mat out_image_;

  // Detector Config Parameter Monitorning 
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_[23];
  std::shared_ptr<rclcpp::ParameterCallbackHandle> callback_handle_[23];

  void paramCallback(const rclcpp::Parameter& p);
};

}       // namespace robotis_op
#endif  // _BALL_DETECTOR_H_
