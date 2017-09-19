/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/* Author: Kayman Jung */

#include <yaml-cpp/yaml.h>
#include <fstream>

#include "ball_detector/ball_detector.h"

namespace robotis_op
{

BallDetector::BallDetector()
    : nh_(ros::this_node::getName()),
      it_(this->nh_),
      enable_(true),
      params_config_(),
      init_param_(false),
      not_found_count_(0)
{
  has_path_ = nh_.getParam("yaml_path", param_path_);

  if (has_path_)
    std::cout << "Path : " << param_path_ << std::endl;

  //detector config struct
  DetectorConfig detect_config;

  //get user parameters from dynamic reconfigure (yaml entries)
  nh_.param<int>("gaussian_blur_size", detect_config.gaussian_blur_size, params_config_.gaussian_blur_size);
  if (detect_config.gaussian_blur_size % 2 == 0)
    detect_config.gaussian_blur_size -= 1;
  if (detect_config.gaussian_blur_size <= 0)
    detect_config.gaussian_blur_size = 1;
  nh_.param<double>("gaussian_blur_sigma", detect_config.gaussian_blur_sigma, params_config_.gaussian_blur_sigma);
  nh_.param<double>("canny_edge_th", detect_config.canny_edge_th, params_config_.canny_edge_th);
  nh_.param<double>("hough_accum_resolution", detect_config.hough_accum_resolution,
                    params_config_.hough_accum_resolution);
  nh_.param<double>("min_circle_dist", detect_config.min_circle_dist, params_config_.min_circle_dist);
  nh_.param<double>("hough_accum_th", detect_config.hough_accum_th, params_config_.hough_accum_th);
  nh_.param<int>("min_radius", detect_config.min_radius, params_config_.min_radius);
  nh_.param<int>("max_radius", detect_config.max_radius, params_config_.max_radius);
  nh_.param<int>("filter_h_min", detect_config.filter_threshold.h_min, params_config_.filter_threshold.h_min);
  nh_.param<int>("filter_h_max", detect_config.filter_threshold.h_max, params_config_.filter_threshold.h_max);
  nh_.param<int>("filter_s_min", detect_config.filter_threshold.s_min, params_config_.filter_threshold.s_min);
  nh_.param<int>("filter_s_max", detect_config.filter_threshold.s_max, params_config_.filter_threshold.s_max);
  nh_.param<int>("filter_v_min", detect_config.filter_threshold.v_min, params_config_.filter_threshold.v_min);
  nh_.param<int>("filter_v_max", detect_config.filter_threshold.v_max, params_config_.filter_threshold.v_max);
  nh_.param<bool>("filter_debug", detect_config.debug, params_config_.debug);
  nh_.param<bool>("use_field", detect_config.use_field, params_config_.use_field);
  nh_.param<int>("field_h_min", detect_config.field_threshold.h_min, params_config_.field_threshold.h_min);
  nh_.param<int>("field_h_max", detect_config.field_threshold.h_max, params_config_.field_threshold.h_max);
  nh_.param<int>("field_s_min", detect_config.field_threshold.s_min, params_config_.field_threshold.s_min);
  nh_.param<int>("field_s_max", detect_config.field_threshold.s_max, params_config_.field_threshold.s_max);
  nh_.param<int>("field_v_min", detect_config.field_threshold.v_min, params_config_.field_threshold.v_min);
  nh_.param<int>("field_v_max", detect_config.field_threshold.v_max, params_config_.field_threshold.v_max);
  nh_.param<bool>("field_debug", detect_config.debug, params_config_.field_debug);
  nh_.param<bool>("use_second_filter", detect_config.use_second_filter, params_config_.use_second_filter);
  nh_.param<int>("filter2_h_min", detect_config.filter2_threshold.h_min, params_config_.filter2_threshold.h_min);
  nh_.param<int>("filter2_h_max", detect_config.filter2_threshold.h_max, params_config_.filter2_threshold.h_max);
  nh_.param<int>("filter2_s_min", detect_config.filter2_threshold.s_min, params_config_.filter2_threshold.s_min);
  nh_.param<int>("filter2_s_max", detect_config.filter2_threshold.s_max, params_config_.filter2_threshold.s_max);
  nh_.param<int>("filter2_v_min", detect_config.filter2_threshold.v_min, params_config_.filter2_threshold.v_min);
  nh_.param<int>("filter2_v_max", detect_config.filter2_threshold.v_max, params_config_.filter2_threshold.v_max);
  nh_.param<int>("ellipse_size", detect_config.ellipse_size, params_config_.ellipse_size);

  //sets publishers
  image_pub_ = it_.advertise("image_out", 100);
  circles_pub_ = nh_.advertise<ball_detector::circleSetStamped>("circle_set", 100);
  camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 100);

  //sets subscribers
  enable_sub_ = nh_.subscribe("enable", 1, &BallDetector::enableCallback, this);
  image_sub_ = it_.subscribe("image_in", 1, &BallDetector::imageCallback, this);
  camera_info_sub_ = nh_.subscribe("cameraInfo_in", 100, &BallDetector::cameraInfoCallback, this);

  //initializes newImageFlag
  new_image_flag_ = false;

  // dynamic_reconfigure
  callback_fnc_ = boost::bind(&BallDetector::dynParamCallback, this, _1, _2);
  param_server_.setCallback(callback_fnc_);

  //sets config and prints it
  params_config_ = detect_config;
  init_param_ = true;
  printConfig();
}

BallDetector::~BallDetector()
{

}

bool BallDetector::newImage()
{
  if (new_image_flag_)
  {
    new_image_flag_ = false;
    return true;
  }
  else
  {
    return false;
  }
}

void BallDetector::process()
{
  if (enable_ == false)
    return;

  if (cv_img_ptr_sub_ != NULL)
  {
    //sets input image
    setInputImage(cv_img_ptr_sub_->image);

    // test image filtering
    filterImage();

    //detect circles
    houghDetection(this->img_encoding_);
  }
}

void BallDetector::publishImage()
{
  if (enable_ == false)
    return;

  //image_raw topic
  cv_img_pub_.header.seq++;
  cv_img_pub_.header.stamp = sub_time_;
  cv_img_pub_.header.frame_id = image_frame_id_;
  switch (img_encoding_)
  {
    case IMG_RGB8:
      cv_img_pub_.encoding = sensor_msgs::image_encodings::RGB8;
      break;
    case IMG_MONO:
      cv_img_pub_.encoding = sensor_msgs::image_encodings::MONO8;
      break;
    default:
      cv_img_pub_.encoding = sensor_msgs::image_encodings::MONO8;
      break;
  }
  getOutputImage(cv_img_pub_.image);
  image_pub_.publish(cv_img_pub_.toImageMsg());
  camera_info_pub_.publish(camera_info_msg_);
}

void BallDetector::publishCircles()
{
  if (enable_ == false)
    return;

  if (circles_.size() == 0)
    return;

  //clears and resize the message
  circles_msg_.circles.clear();
  circles_msg_.circles.resize(circles_.size());

  //fill header
  circles_msg_.header.seq++;
  circles_msg_.header.stamp = sub_time_;
  circles_msg_.header.frame_id = "detector";  //To do: get frame_id from input image

  //fill circle data
  // top(-1), bottom(+1)
  // left(-1), right(+1)
  for (int idx = 0; idx < circles_.size(); idx++)
  {
    circles_msg_.circles[idx].x = circles_[idx][0] / in_image_.cols * 2 - 1;    // x (-1 ~ 1)
    circles_msg_.circles[idx].y = circles_[idx][1] / in_image_.rows * 2 - 1;    // y (-1 ~ 1)
    circles_msg_.circles[idx].z = circles_[idx][2];    // radius
  }

  //publish message
  circles_pub_.publish(circles_msg_);
}

void BallDetector::enableCallback(const std_msgs::Bool::ConstPtr &msg)
{
  enable_ = msg->data;
}

void BallDetector::imageCallback(const sensor_msgs::ImageConstPtr & msg)
{
  if (enable_ == false)
    return;

  try
  {
    if (msg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0)
      this->img_encoding_ = IMG_MONO;
    if (msg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0)
      this->img_encoding_ = IMG_RGB8;
    this->cv_img_ptr_sub_ = cv_bridge::toCvCopy(msg, msg->encoding);
  } catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  //indicates a new image is available
  this->sub_time_ = msg->header.stamp;
  this->image_frame_id_ = msg->header.frame_id;
  this->new_image_flag_ = true;
  return;
}

void BallDetector::dynParamCallback(ball_detector::detectorParamsConfig &config, uint32_t level)
{
  params_config_.gaussian_blur_size = config.gaussian_blur_size;
  params_config_.gaussian_blur_sigma = config.gaussian_blur_sigma;
  params_config_.canny_edge_th = config.canny_edge_th;
  params_config_.hough_accum_resolution = config.hough_accum_resolution;
  params_config_.min_circle_dist = config.min_circle_dist;
  params_config_.hough_accum_th = config.hough_accum_th;
  params_config_.min_radius = config.min_radius;
  params_config_.max_radius = config.max_radius;
  params_config_.filter_threshold.h_min = config.filter_h_min;
  params_config_.filter_threshold.h_max = config.filter_h_max;
  params_config_.filter_threshold.s_min = config.filter_s_min;
  params_config_.filter_threshold.s_max = config.filter_s_max;
  params_config_.filter_threshold.v_min = config.filter_v_min;
  params_config_.filter_threshold.v_max = config.filter_v_max;
  params_config_.debug = config.debug_image;
  params_config_.use_field = config.use_field;
  params_config_.field_threshold.h_min = config.filter2_h_min;
  params_config_.field_threshold.h_max = config.filter2_h_max;
  params_config_.field_threshold.s_min = config.filter2_s_min;
  params_config_.field_threshold.s_max = config.filter2_s_max;
  params_config_.field_threshold.v_min = config.filter2_v_min;
  params_config_.field_threshold.v_max = config.filter2_v_max;
  params_config_.field_debug = config.field_debug_image;
  params_config_.use_second_filter = config.use_second_filter;
  params_config_.filter2_threshold.h_min = config.filter2_h_min;
  params_config_.filter2_threshold.h_max = config.filter2_h_max;
  params_config_.filter2_threshold.s_min = config.filter2_s_min;
  params_config_.filter2_threshold.s_max = config.filter2_s_max;
  params_config_.filter2_threshold.v_min = config.filter2_v_min;
  params_config_.filter2_threshold.v_max = config.filter2_v_max;
  params_config_.ellipse_size = config.ellipse_size;

  // gaussian_blur has to be odd number.
  if (params_config_.gaussian_blur_size % 2 == 0)
    params_config_.gaussian_blur_size -= 1;
  if (params_config_.gaussian_blur_size <= 0)
    params_config_.gaussian_blur_size = 1;

  printConfig();
  saveConfig();
}

void BallDetector::cameraInfoCallback(const sensor_msgs::CameraInfo & msg)
{
  if (enable_ == false)
    return;

  camera_info_msg_ = msg;
}

void BallDetector::printConfig()
{
  if (init_param_ == false)
    return;

  std::cout << "Detetctor Configuration:" << std::endl
            << "    gaussian_blur_size: " << params_config_.gaussian_blur_size << std::endl
            << "    gaussian_blur_sigma: " << params_config_.gaussian_blur_sigma << std::endl
            << "    canny_edge_th: " << params_config_.canny_edge_th << std::endl
            << "    hough_accum_resolution: " << params_config_.hough_accum_resolution << std::endl
            << "    min_circle_dist: " << params_config_.min_circle_dist << std::endl
            << "    hough_accum_th: " << params_config_.hough_accum_th << std::endl
            << "    min_radius: " << params_config_.min_radius << std::endl
            << "    max_radius: " << params_config_.max_radius << std::endl
            << "    filter_h_min: " << params_config_.filter_threshold.h_min << std::endl
            << "    filter_h_max: " << params_config_.filter_threshold.h_max << std::endl
            << "    filter_s_min: " << params_config_.filter_threshold.s_min << std::endl
            << "    filter_s_max: " << params_config_.filter_threshold.s_max << std::endl
            << "    filter_v_min: " << params_config_.filter_threshold.v_min << std::endl
            << "    filter_v_max: " << params_config_.filter_threshold.v_max << std::endl
            << "    filter_image_to_debug: " << params_config_.debug << std::endl
            << "    field_filter: " << params_config_.use_field << std::endl
            << "    field_h_min: " << params_config_.field_threshold.h_min << std::endl
            << "    field_h_max: " << params_config_.field_threshold.h_max << std::endl
            << "    field_s_min: " << params_config_.field_threshold.s_min << std::endl
            << "    field_s_max: " << params_config_.field_threshold.s_max << std::endl
            << "    field_v_min: " << params_config_.field_threshold.v_min << std::endl
            << "    field_v_max: " << params_config_.field_threshold.v_max << std::endl
            << "    filter_image_for_field: " << params_config_.field_debug << std::endl
            << "    use_second_filter: " << params_config_.use_second_filter << std::endl
            << "    filter2_h_min: " << params_config_.filter2_threshold.h_min << std::endl
            << "    filter2_h_max: " << params_config_.filter2_threshold.h_max << std::endl
            << "    filter2_s_min: " << params_config_.filter2_threshold.s_min << std::endl
            << "    filter2_s_max: " << params_config_.filter2_threshold.s_max << std::endl
            << "    filter2_v_min: " << params_config_.filter2_threshold.v_min << std::endl
            << "    filter2_v_max: " << params_config_.filter2_threshold.v_max << std::endl
            << "    ellipse_size: " << params_config_.ellipse_size << std::endl << std::endl;
}

void BallDetector::saveConfig()
{
  if (has_path_ == false)
    return;

  YAML::Emitter yaml_out;

  yaml_out << YAML::BeginMap;
  yaml_out << YAML::Key << "gaussian_blur_size" << YAML::Value << params_config_.gaussian_blur_size;
  yaml_out << YAML::Key << "gaussian_blur_sigma" << YAML::Value << params_config_.gaussian_blur_sigma;
  yaml_out << YAML::Key << "canny_edge_th" << YAML::Value << params_config_.canny_edge_th;
  yaml_out << YAML::Key << "hough_accum_resolution" << YAML::Value << params_config_.hough_accum_resolution;
  yaml_out << YAML::Key << "min_circle_dist" << YAML::Value << params_config_.min_circle_dist;
  yaml_out << YAML::Key << "hough_accum_th" << YAML::Value << params_config_.hough_accum_th;
  yaml_out << YAML::Key << "min_radius" << YAML::Value << params_config_.min_radius;
  yaml_out << YAML::Key << "max_radius" << YAML::Value << params_config_.max_radius;
  yaml_out << YAML::Key << "filter_h_min" << YAML::Value << params_config_.filter_threshold.h_min;
  yaml_out << YAML::Key << "filter_h_max" << YAML::Value << params_config_.filter_threshold.h_max;
  yaml_out << YAML::Key << "filter_s_min" << YAML::Value << params_config_.filter_threshold.s_min;
  yaml_out << YAML::Key << "filter_s_max" << YAML::Value << params_config_.filter_threshold.s_max;
  yaml_out << YAML::Key << "filter_v_min" << YAML::Value << params_config_.filter_threshold.v_min;
  yaml_out << YAML::Key << "filter_v_max" << YAML::Value << params_config_.filter_threshold.v_max;
  yaml_out << YAML::Key << "filter_debug" << YAML::Value << params_config_.debug;
  yaml_out << YAML::Key << "use_field" << YAML::Value << params_config_.use_field;
  yaml_out << YAML::Key << "field_h_min" << YAML::Value << params_config_.field_threshold.h_min;
  yaml_out << YAML::Key << "field_h_max" << YAML::Value << params_config_.field_threshold.h_max;
  yaml_out << YAML::Key << "field_s_min" << YAML::Value << params_config_.field_threshold.s_min;
  yaml_out << YAML::Key << "field_s_max" << YAML::Value << params_config_.field_threshold.s_max;
  yaml_out << YAML::Key << "field_v_min" << YAML::Value << params_config_.field_threshold.v_min;
  yaml_out << YAML::Key << "field_v_max" << YAML::Value << params_config_.field_threshold.v_max;
  yaml_out << YAML::Key << "field_debug" << YAML::Value << params_config_.field_debug;
  yaml_out << YAML::Key << "use_second_filter" << YAML::Value << params_config_.use_second_filter;
  yaml_out << YAML::Key << "filter2_h_min" << YAML::Value << params_config_.filter2_threshold.h_min;
  yaml_out << YAML::Key << "filter2_h_max" << YAML::Value << params_config_.filter2_threshold.h_max;
  yaml_out << YAML::Key << "filter2_s_min" << YAML::Value << params_config_.filter2_threshold.s_min;
  yaml_out << YAML::Key << "filter2_s_max" << YAML::Value << params_config_.filter2_threshold.s_max;
  yaml_out << YAML::Key << "filter2_v_min" << YAML::Value << params_config_.filter2_threshold.v_min;
  yaml_out << YAML::Key << "filter2_v_max" << YAML::Value << params_config_.filter2_threshold.v_max;
  yaml_out << YAML::Key << "ellipse_size" << YAML::Value << params_config_.ellipse_size;
  yaml_out << YAML::EndMap;

  // output to file
  std::ofstream fout(param_path_.c_str());
  fout << yaml_out.c_str();
}

void BallDetector::setInputImage(const cv::Mat & inIm)
{
  in_image_ = inIm.clone();

  if (params_config_.debug == false)
    out_image_ = in_image_.clone();
}

void BallDetector::getOutputImage(cv::Mat & outIm)
{
  this->drawOutputImage();
  outIm = out_image_.clone();
}

void BallDetector::filterImage()
{
  if (!in_image_.data)
    return;

  cv::Mat img_hsv, img_filtered;
  cv::cvtColor(in_image_, img_hsv, cv::COLOR_RGB2HSV);

  inRangeHsv(img_hsv, params_config_.filter_threshold, img_filtered);

  // morphology : open and close
  morphology(img_filtered, img_filtered, params_config_.ellipse_size);

  if (params_config_.use_second_filter == true)
  {
    // mask
    cv::Mat img_mask;

    // check hsv range
    //cv::Mat img_filtered2;
    inRangeHsv(img_hsv, params_config_.filter2_threshold, img_filtered2);

    makeFilterMaskFromBall(img_filtered, img_mask);
    cv::bitwise_and(img_filtered2, img_mask, img_filtered2);

    // or
    cv::bitwise_or(img_filtered, img_filtered2, img_filtered);
  }

  if (params_config_.use_field == true)
  {
    // mask
    cv::Mat img_mask;

    //hsv range
    inRangeHsv(img_hsv, params_config_.field_threshold, img_field_filtered);
    makeFilterMaskFromBall(img_filtered, img_mask);
    cv::bitwise_and(img_field_filtered, img_mask, img_filtered);

    cv::cvtColor(img_field_filtered, img_field_filtered, cv::COLOR_GRAY2RGB);
  }

  morphology(img_filtered, img_filtered, params_config_.ellipse_size);

  cv::cvtColor(img_filtered, in_image_, cv::COLOR_GRAY2RGB);
}

void BallDetector::makeFilterMask(const cv::Mat &source_img, cv::Mat &mask_img, int range)
{
  // source_img.
  mask_img = cv::Mat::zeros(source_img.rows, source_img.cols, CV_8UC1);

  int source_height = source_img.rows;
  int source_width = source_img.cols;

  // channel : 1
  if (source_img.channels() != 1)
    return;

  for (int i = 0; i < source_height; i++)
  {
    for (int j = 0; j < source_width; j++)
    {
      uint8_t pixel = source_img.at < uint8_t > (i, j);

      if (pixel == 0)
        continue;

      for (int mask_i = i - range; mask_i <= i + range; mask_i++)
      {
        if (mask_i < 0 || mask_i >= source_height)
          continue;

        for (int mask_j = j - range; mask_j <= j + range; mask_j++)
        {
          if (mask_j < 0 || mask_j >= source_width)
            continue;

          mask_img.at < uchar > (mask_i, mask_j, 0) = 255;
        }
      }
    }
  }
}

void BallDetector::makeFilterMaskFromBall(const cv::Mat &source_img, cv::Mat &mask_img)
{
  // source_img.
  mask_img = cv::Mat::zeros(source_img.rows, source_img.cols, CV_8UC1);

  if (circles_.size() == 0)
    return;

  // channel : 1
  if (source_img.channels() != 1)
    return;

  cv::Mat img_labels, stats, centroids;
  int numOfLables = cv::connectedComponentsWithStats(source_img, img_labels, stats, centroids, 8, CV_32S);
  for (int j = 1; j < numOfLables; j++)
  {
    int area = stats.at<int>(j, cv::CC_STAT_AREA);
    int left = stats.at<int>(j, cv::CC_STAT_LEFT);
    int top = stats.at<int>(j, cv::CC_STAT_TOP);
    int width = stats.at<int>(j, cv::CC_STAT_WIDTH);
    int height = stats.at<int>(j, cv::CC_STAT_HEIGHT);

    int center_x = left + width * 0.5;
    int center_y = top + height * 0.5;
    int radius = (width + height) * 0.5;

    for (int mask_i = center_y - radius; mask_i <= center_y + radius; mask_i++)
    {
      if (mask_i < 0 || mask_i >= source_img.rows)
        continue;

      int mask_offset = abs(mask_i - center_y) * 0.5;

      for (int mask_j = center_x - radius + mask_offset; mask_j <= center_x + radius - mask_offset; mask_j++)
      {
        if (mask_j < 0 || mask_j >= source_img.cols)
          continue;

        mask_img.at < uchar > (mask_i, mask_j, 0) = 255;
      }
    }
  }

}

void BallDetector::inRangeHsv(const cv::Mat &input_img, const HsvFilter &filter_value, cv::Mat &output_img)
{
  // 0-360 -> 0-180
  int scaled_hue_min = static_cast<int>(filter_value.h_min * 0.5);
  int scaled_hue_max = static_cast<int>(filter_value.h_max * 0.5);

  if (scaled_hue_min <= scaled_hue_max)
  {
    cv::Scalar min_value = cv::Scalar(scaled_hue_min, filter_value.s_min, filter_value.v_min, 0);
    cv::Scalar max_value = cv::Scalar(scaled_hue_max, filter_value.s_max, filter_value.v_max, 0);

    cv::inRange(input_img, min_value, max_value, output_img);
  }
  else
  {
    cv::Mat lower_hue_range, upper_hue_range;
    cv::Scalar min_value, max_value;

    min_value = cv::Scalar(0, filter_value.s_min, filter_value.v_min, 0);
    max_value = cv::Scalar(scaled_hue_max, filter_value.s_max, filter_value.v_max, 0);
    cv::inRange(input_img, min_value, max_value, lower_hue_range);

    min_value = cv::Scalar(scaled_hue_min, filter_value.s_min, filter_value.v_min, 0);
    max_value = cv::Scalar(179, filter_value.s_max, filter_value.v_max, 0);
    cv::inRange(input_img, min_value, max_value, upper_hue_range);

    cv::bitwise_or(lower_hue_range, upper_hue_range, output_img);
  }
}

void BallDetector::morphology(const cv::Mat &intput_img, cv::Mat &output_img, int ellipse_size)
{
  cv::erode(intput_img, output_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ellipse_size, ellipse_size)));
  cv::dilate(output_img, output_img,
             cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ellipse_size * 2, ellipse_size * 2)));

  cv::dilate(output_img, output_img,
             cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ellipse_size, ellipse_size)));
  cv::erode(output_img, output_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ellipse_size, ellipse_size)));
}

void BallDetector::houghDetection(const unsigned int imgEncoding)
{
  cv::Mat gray_image;
  std::vector<cv::Vec3f> circles_current;
  std::vector<cv::Vec3f> prev_circles = circles_;

  //clear previous circles
  circles_.clear();

  // If input image is RGB, convert it to gray
  if (imgEncoding == IMG_RGB8)
    cv::cvtColor(in_image_, gray_image, CV_RGB2GRAY);

  //Reduce the noise so we avoid false circle detection
  cv::GaussianBlur(gray_image, gray_image,
                   cv::Size(params_config_.gaussian_blur_size, params_config_.gaussian_blur_size),
                   params_config_.gaussian_blur_sigma);

  double hough_accum_th = params_config_.hough_accum_th;


  //Apply the Hough Transform to find the circles
  cv::HoughCircles(gray_image, circles_current, CV_HOUGH_GRADIENT, params_config_.hough_accum_resolution,
                   params_config_.min_circle_dist, params_config_.canny_edge_th, hough_accum_th,
                   params_config_.min_radius, params_config_.max_radius);

  if (circles_current.size() == 0)
    not_found_count_ += 1;
  else
    not_found_count_ = 0;

  double alpha = 0.2;

  for (int ix = 0; ix < circles_current.size(); ix++)
  {
    cv::Point2d center = cv::Point(circles_current[ix][0], circles_current[ix][1]);
    double radius = circles_current[ix][2];

    for (int prev_ix = 0; prev_ix < prev_circles.size(); prev_ix++)
    {
      cv::Point2d prev_center = cv::Point(prev_circles[prev_ix][0], prev_circles[prev_ix][1]);
      double prev_radius = prev_circles[prev_ix][2];

      cv::Point2d diff = center - prev_center;
      double radius_th = std::max(radius, prev_radius) * 0.75;

      if (sqrt(diff.dot(diff)) < radius_th)
      {
        if (abs(radius - prev_radius) < radius_th)
        {
          circles_current[ix] = circles_current[ix] * alpha + prev_circles[prev_ix] * (1 - alpha);
        }

        break;
      }
    }

    circles_.push_back(circles_current[ix]);
  }
}

void BallDetector::drawOutputImage()
{
  cv::Point center_position;
  int radius = 0;
  size_t ii;

  //draws results to output Image
  if (params_config_.debug == true)
  {
    out_image_ = in_image_.clone();
//    if (params_config_.field_debug == true)
//      out_image_ = img_field_filtered.clone();
  }

  for (ii = 0; ii < circles_.size(); ii++)
  {
    {
      int this_radius = cvRound(circles_[ii][2]);
      if (this_radius > radius)
      {
        radius = this_radius;
        center_position = cv::Point(cvRound(circles_[ii][0]), cvRound(circles_[ii][1]));
      }
    }
  }
  cv::circle(out_image_, center_position, 5, cv::Scalar(0, 0, 255), -1, 8, 0);      // circle center in blue
  cv::circle(out_image_, center_position, radius, cv::Scalar(0, 0, 255), 3, 8, 0);      // circle outline in blue

}

}       // namespace robotis_op
