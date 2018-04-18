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

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include "op3_ball_detector/ball_detector_config.h"

#include "op3_ball_detector/DetectorParamsConfig.h"
#include "op3_ball_detector/CircleSetStamped.h"
#include "op3_ball_detector/GetParameters.h"
#include "op3_ball_detector/SetParameters.h"

namespace robotis_op
{

class BallDetector
{
 public:
  BallDetector();
  ~BallDetector();

  //checks if a new image has been received
  bool newImage();

  //execute circle detection with the cureent image
  void process();

  //publish the output image (input image + marked circles)
  void publishImage();

  //publish the circle set data
  void publishCircles();

 protected:
  const static int NOT_FOUND_TH = 30;

  //callbacks to image subscription
  void imageCallback(const sensor_msgs::ImageConstPtr & msg);

  //callbacks to camera info subscription
  void cameraInfoCallback(const sensor_msgs::CameraInfo & msg);

  void dynParamCallback(op3_ball_detector::DetectorParamsConfig &config, uint32_t level);
  void enableCallback(const std_msgs::Bool::ConstPtr &msg);

  void paramCommandCallback(const std_msgs::String::ConstPtr &msg);
  bool setParamCallback(op3_ball_detector::SetParameters::Request &req, op3_ball_detector::SetParameters::Response &res);
  bool getParamCallback(op3_ball_detector::GetParameters::Request &req, op3_ball_detector::GetParameters::Response &res);
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

  //ros node handle
  ros::NodeHandle nh_;

  ros::Subscriber enable_sub_;

  //image publisher/subscriber
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
  cv_bridge::CvImage cv_img_pub_;
  image_transport::Subscriber image_sub_;
  cv_bridge::CvImagePtr cv_img_ptr_sub_;

  bool enable_;
  bool init_param_;
  int not_found_count_;

  //circle set publisher
  op3_ball_detector::CircleSetStamped circles_msg_;
  ros::Publisher circles_pub_;

  //camera info subscriber
  sensor_msgs::CameraInfo camera_info_msg_;
  ros::Subscriber camera_info_sub_;
  ros::Publisher camera_info_pub_;

  //dynamic reconfigure
  DetectorConfig params_config_;
  std::string param_path_;
  bool has_path_;

  // web setting
  std::string default_setting_path_;
  ros::Publisher param_pub_;
  ros::Subscriber param_command_sub_;
  ros::ServiceServer get_param_client_;
  ros::ServiceServer set_param_client_;

  //flag indicating a new image has been received
  bool new_image_flag_;

  //image time stamp and frame id
  ros::Time sub_time_;
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

  dynamic_reconfigure::Server<op3_ball_detector::DetectorParamsConfig> param_server_;
  dynamic_reconfigure::Server<op3_ball_detector::DetectorParamsConfig>::CallbackType callback_fnc_;
};

}       // namespace robotis_op
#endif  // _BALL_DETECTOR_H_
