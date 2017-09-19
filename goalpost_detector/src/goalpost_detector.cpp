/* Author: Seri Lee */

#include <yaml-cpp/yaml.h>
#include <fstream>

#include "goalpost_detector/goalpost_detector.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace robotis_op
{

GoalpostDetector::GoalpostDetector()
  : nh_(ros::this_node::getName()),
    it_(this->nh_),
    enable_(true),
    params_config_(),
    init_param_(false),
    RHO (1.0),
    THETA (1.0)

{
  has_path_ = nh_.getParam("yaml_path", param_path_);

  if (has_path_)
    std::cout << "Path : " << param_path_ << std::endl;

  //detector config struct
  PostDetectorConfig detect_config;

  //get user parameters from dynamic reconfigure (yaml entries)
  nh_.param<int>("gaussian_blur_size", detect_config.gaussian_blur_size, params_config_.gaussian_blur_size);
  if (detect_config.gaussian_blur_size % 2 == 0)
    detect_config.gaussian_blur_size -= 1;
  if (detect_config.gaussian_blur_size <= 0)
    detect_config.gaussian_blur_size = 1;
  nh_.param<double>("gaussian_blur_sigma", detect_config.gaussian_blur_sigma, params_config_.gaussian_blur_sigma);
  nh_.param<double>("canny_th_low", detect_config.canny_th_low, params_config_.canny_th_low);
  nh_.param<double>("canny_th_high", detect_config.canny_th_high, params_config_.canny_th_high);
  nh_.param<double>("min_line_length", detect_config.min_line_length, params_config_.min_line_length);
  nh_.param<double>("max_line_gap", detect_config.max_line_gap, params_config_.max_line_gap);
  nh_.param<double>("hough_accum_th", detect_config.hough_accum_th, params_config_.hough_accum_th);
  nh_.param<int>("goalpost_h_min", detect_config.goalpost_threshold.h_min, params_config_.goalpost_threshold.h_min);
  nh_.param<int>("goalpost_h_max", detect_config.goalpost_threshold.h_max, params_config_.goalpost_threshold.h_max);
  nh_.param<int>("goalpost_s_min", detect_config.goalpost_threshold.s_min, params_config_.goalpost_threshold.s_min);
  nh_.param<int>("goalpost_s_max", detect_config.goalpost_threshold.s_max, params_config_.goalpost_threshold.s_max);
  nh_.param<int>("goalpost_v_min", detect_config.goalpost_threshold.v_min, params_config_.goalpost_threshold.v_min);
  nh_.param<int>("goalpost_v_max", detect_config.goalpost_threshold.v_max, params_config_.goalpost_threshold.v_max);
  nh_.param<bool>("field_filter", detect_config.field_filter, params_config_.field_filter);
  nh_.param<int>("field_h_min", detect_config.field_threshold.h_min, params_config_.field_threshold.h_min);
  nh_.param<int>("field_h_max", detect_config.field_threshold.h_max, params_config_.field_threshold.h_max);
  nh_.param<int>("field_s_min", detect_config.field_threshold.s_min, params_config_.field_threshold.s_min);
  nh_.param<int>("field_s_max", detect_config.field_threshold.s_max, params_config_.field_threshold.s_max);
  nh_.param<int>("field_v_min", detect_config.field_threshold.v_min, params_config_.field_threshold.v_min);
  nh_.param<int>("field_v_max", detect_config.field_threshold.v_max, params_config_.field_threshold.v_max);
  nh_.param<int>("ellipse_size", detect_config.ellipse_size, params_config_.ellipse_size);
  nh_.param<int>("gain", detect_config.gain, params_config_.gain);
  nh_.param<int>("brightness", detect_config.brightness, params_config_.brightness);
  nh_.param<int>("exposure", detect_config.exposure, params_config_.exposure);
  nh_.param<bool>("filter_debug", detect_config.debug, params_config_.debug);

  //sets publishers
  image_pub_ = it_.advertise("image_out", 100);
  camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 100);
  lines_pub_ = nh_.advertise<goalpost_detector::LineSetStamped>("line_set", 100);
  camera_params_pub_ = nh_.advertise<op3_camera_setting_tool::V4lParameters>("/op3_camera/set_params",100);

  //sets subscribers
  enable_sub_ = nh_.subscribe("enable", 1, &GoalpostDetector::enableCallback, this);
  image_sub_ = it_.subscribe("image_in", 1, &GoalpostDetector::imageCallback, this);
  camera_info_sub_ = nh_.subscribe("cameraInfo_in", 100, &GoalpostDetector::cameraInfoCallback, this);

  //initializes newImageFlag
  new_image_flag_ = false;

  // dynamic_reconfigure //bind save serve to rqt
  callback_fnc_ = boost::bind(&GoalpostDetector::dynParamCallback, this, _1, _2);
  param_server_.setCallback(callback_fnc_);

  //sets config and prints it
  params_config_ = detect_config;
  init_param_ = true;
  printConfig();

}
GoalpostDetector::~GoalpostDetector()
{

}

bool GoalpostDetector::newImage()
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

void GoalpostDetector::process()
{
  if(enable_ == false)
    return;

  if (cv_img_ptr_sub_ != NULL)
  {
    //sets input image
    setInputImage(cv_img_ptr_sub_->image);

    // goalpost filtering
    filterImage();

    // detecting lines
    houghDetection(img_encoding_);

    // get key points
    getKeyPoints();
  }
}

//void GoalpostDetector::publishParams()
//{
////  std_msgs::Bool msgs;
////  msgs.data = param_server_;
////  camera_params_pub_.publish(msgs);
//  goalpost_detector::GoalpostDetectorParamsConfig->publishMessage(&brightness_pub_);
//}

void GoalpostDetector::publishImage()
{
  if(enable_ == false)
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
  //draw lines and get OutputImage
  getOutputImage(cv_img_pub_.image);

  //Publish Image and Camera Information
  image_pub_.publish(cv_img_pub_.toImageMsg());
  camera_info_pub_.publish(camera_info_msg_);
}

void GoalpostDetector::publishLines()
{
  if(enable_ == false)
    return;

  if (filtered_lines_.size() == 0)
    return;

  //clears and resize the message
  lines_msg_.lines.clear();
  lines_msg_.lines.resize(filtered_lines_.size());

  //fill header
  lines_msg_.header.seq++;
  lines_msg_.header.stamp = sub_time_;
  lines_msg_.header.frame_id = "detector";  //get frame_id from input image

    for(size_t i = 0; i < filtered_lines_.size(); i++ )
    {
      float r = filtered_lines_[i][0], t = filtered_lines_[i][1];
      double x0 = r * cos(t), y0 = r * sin(t);
      double alpha = 1000;

      cv::Point pt1( cvRound( x0 + alpha*(-sin(t)) ), cvRound(y0 + alpha*cos(t)) );
      cv::Point pt2( cvRound( x0 - alpha*(-sin(t)) ), cvRound(y0 - alpha*cos(t)) );

      goalpost_detector::Line line_msg;

      line_msg.pt1.x = pt1.x;
      line_msg.pt1.y = pt1.y;
      line_msg.pt2.x = pt2.x;
      line_msg.pt2.y = pt2.y;

      lines_msg_.lines.push_back(line_msg);
      lines_pub_.publish(lines_msg_);
    }

//  ///////for HoughLines_Vertical Lines///////
//  for(size_t i = 0; i < filtered_lines_v.size(); i++ )
//  {
//    float r = filtered_lines_v[i][0], t = filtered_lines_v[i][1];
//    double x0 = r * cos(t), y0 = r * sin(t);
//    double alpha = 1000;

//    cv::Point pt1( cvRound( x0 + alpha*(-sin(t)) ), cvRound(y0 + alpha*cos(t)) );
//    cv::Point pt2( cvRound( x0 - alpha*(-sin(t)) ), cvRound(y0 - alpha*cos(t)) );

//    goalpost_detector::Line line_msg;

//    line_msg.pt1.x = pt1.x;
//    line_msg.pt1.y = pt1.y;
//    line_msg.pt2.x = pt2.x;
//    line_msg.pt2.y = pt2.y;

//    lines_msg_.lines.push_back(line_msg);
//    lines_v_pub_.publish(lines_msg_);
//  }

//  /////for HoughLines_Horizontal Lines///////
//  for(size_t i = 0; i < filtered_lines_h.size(); i++ )
//  {
//    float r1 = filtered_lines_h[i][0], t1 = filtered_lines_h[i][1];
//    double x1 = r1 * cos(t1), y1 = r1 * sin(t1);
//    double alpha = 1000;

//    cv::Point pt1( cvRound( x1 + alpha*(-sin(t1)) ), cvRound(y1 + alpha*cos(t1)) );
//    cv::Point pt2( cvRound( x1 - alpha*(-sin(t1)) ), cvRound(y1 - alpha*cos(t1)) );

//    goalpost_detector::Line line_msg;

//    line_msg.pt1.x = pt1.x;
//    line_msg.pt1.y = pt1.y;
//    line_msg.pt2.x = pt2.x;
//    line_msg.pt2.y = pt2.y;

//    lines_msg_.lines.push_back(line_msg);
//    lines_h_pub_.publish(lines_msg_);
//  }

  ///////for HoughLinesP///////
//  for(size_t i = 0; i < filtered_lines.size(); i++ )
//  {
//    cv::Vec4i l = filtered_lines[i];

//    goalpost_detector::Line line_msg;
//    line_msg.pt1.x = l[0];
//    line_msg.pt1.y = l[1];
//    line_msg.pt2.x = l[2];
//    line_msg.pt2.y = l[3];

//    lines_msg_.lines.push_back(line_msg);
//  }
}

void GoalpostDetector::enableCallback(const std_msgs::Bool::ConstPtr &msg)
{
  enable_ = msg->data;
}

void GoalpostDetector::imageCallback(const sensor_msgs::ImageConstPtr & msg)
{
  if(enable_ == false)
    return;

  try
  {
    if (msg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0)
      img_encoding_ = IMG_MONO;
    if (msg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0)
      img_encoding_ = IMG_RGB8;
    cv_img_ptr_sub_ = cv_bridge::toCvCopy(msg, msg->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  //indicates a new image is available
  sub_time_ = msg->header.stamp;
  image_frame_id_ = msg->header.frame_id;
  new_image_flag_ = true;
  return;
}

void GoalpostDetector::cameraInfoCallback(const sensor_msgs::CameraInfo & msg)
{
  if(enable_ == false)
    return;

  camera_info_msg_ = msg;
}

void GoalpostDetector::dynParamCallback(goalpost_detector::GoalpostDetectorParamsConfig &config, uint32_t level)
{
  params_config_.gaussian_blur_size = config.gaussian_blur_size;
  params_config_.gaussian_blur_sigma = config.gaussian_blur_sigma;
  params_config_.canny_th_low = config.canny_th_low;
  params_config_.canny_th_high = config.canny_th_high;
  params_config_.min_line_length = config.min_line_length;
  params_config_.hough_accum_th = config.hough_accum_th;
  params_config_.max_line_gap = config.max_line_gap;
  params_config_.goalpost_threshold.h_min = config.goalpost_h_min;
  params_config_.goalpost_threshold.h_max = config.goalpost_h_max;
  params_config_.goalpost_threshold.s_min = config.goalpost_s_min;
  params_config_.goalpost_threshold.s_max = config.goalpost_s_max;
  params_config_.goalpost_threshold.v_min = config.goalpost_v_min;
  params_config_.goalpost_threshold.v_max = config.goalpost_v_max;

  params_config_.field_filter = config.field_filter;
  params_config_.field_threshold.h_min = config.field_h_min;
  params_config_.field_threshold.h_max = config.field_h_max;
  params_config_.field_threshold.s_min = config.field_s_min;
  params_config_.field_threshold.s_max = config.field_s_max;
  params_config_.field_threshold.v_min = config.field_v_min;
  params_config_.field_threshold.v_max = config.field_v_max;

  params_config_.ellipse_size = config.ellipse_size;
  params_config_.debug = config.debug_image;

  params_config_.gain = config.gain;
  params_config_.brightness = config.brightness;
  params_config_.exposure = config.exposure;


  // gaussian_blur has to be odd number.
  if (params_config_.gaussian_blur_size % 2 == 0)
    params_config_.gaussian_blur_size -= 1;
  if (params_config_.gaussian_blur_size <= 0)
    params_config_.gaussian_blur_size = 1;

  printConfig();
  saveConfig();
  publishConfig();
}

void GoalpostDetector::printConfig()
{
  if (init_param_ == false)
    return;

  std::cout << "GoalpostDetetctor Configuration:" << std::endl
            << "    gaussian_blur_size: " << params_config_.gaussian_blur_size << std::endl
            << "    gaussian_blur_sigma: " << params_config_.gaussian_blur_sigma << std::endl
            << "    canny_th_low: " << params_config_.canny_th_low << std::endl
            << "    canny_th_high:" << params_config_.canny_th_high << std::endl
            << "    min_line_length: " << params_config_.min_line_length << std::endl
            << "    max_line_gap: " << params_config_.max_line_gap << std::endl
            << "    hough_accum_th: " << params_config_.hough_accum_th << std::endl
            << "    goalpost_h_min: " << params_config_.goalpost_threshold.h_min << std::endl
            << "    goalpost_h_max: " << params_config_.goalpost_threshold.h_max << std::endl
            << "    goalpost_s_min: " << params_config_.goalpost_threshold.s_min << std::endl
            << "    goalpost_s_max: " << params_config_.goalpost_threshold.s_max << std::endl
            << "    goalpost_v_min: " << params_config_.goalpost_threshold.v_min << std::endl
            << "    goalpost_v_max: " << params_config_.goalpost_threshold.v_max << std::endl
            << "    filter_for_field: " << params_config_.field_filter << std::endl
            << "    field_h_min: " << params_config_.field_threshold.h_min << std::endl
            << "    field_h_max: " << params_config_.field_threshold.h_max << std::endl
            << "    field_s_min: " << params_config_.field_threshold.s_min << std::endl
            << "    field_s_max: " << params_config_.field_threshold.s_max << std::endl
            << "    field_v_min: " << params_config_.field_threshold.v_min << std::endl
            << "    field_v_max: " << params_config_.field_threshold.v_max << std::endl
            << "    ellipse_size: " << params_config_.ellipse_size << std::endl
            << "    gain: " << params_config_.gain << std::endl
            << "    brightness: " << params_config_.brightness << std::endl
            << "    exposure: " << params_config_.exposure << std::endl
            << "    filter_image_to_debug: " << params_config_.debug << std::endl << std::endl;
}

void GoalpostDetector::saveConfig()
{
  if (has_path_ == false)
    return;

  YAML::Emitter yaml_out;

  yaml_out << YAML::BeginMap;
  yaml_out << YAML::Key << "gaussian_blur_size" << YAML::Value << params_config_.gaussian_blur_size;
  yaml_out << YAML::Key << "gaussian_blur_sigma" << YAML::Value << params_config_.gaussian_blur_sigma;
  yaml_out << YAML::Key << "canny_th_low" << YAML::Value << params_config_.canny_th_low;
  yaml_out << YAML::Key << "canny_th_high" << YAML::Value << params_config_.canny_th_high;
  yaml_out << YAML::Key << "min_line_length" << YAML::Value << params_config_.min_line_length;
  yaml_out << YAML::Key << "max_line_gap" << YAML::Value << params_config_.max_line_gap;
  yaml_out << YAML::Key << "hough_accum_th" << YAML::Value << params_config_.hough_accum_th;
  yaml_out << YAML::Key << "goalpost_h_min" << YAML::Value << params_config_.goalpost_threshold.h_min;
  yaml_out << YAML::Key << "goalpost_h_max" << YAML::Value << params_config_.goalpost_threshold.h_max;
  yaml_out << YAML::Key << "goalpost_s_min" << YAML::Value << params_config_.goalpost_threshold.s_min;
  yaml_out << YAML::Key << "goalpost_s_max" << YAML::Value << params_config_.goalpost_threshold.s_max;
  yaml_out << YAML::Key << "goalpost_v_min" << YAML::Value << params_config_.goalpost_threshold.v_min;
  yaml_out << YAML::Key << "goalpost_v_max" << YAML::Value << params_config_.goalpost_threshold.v_max;
  yaml_out << YAML::Key << "field_filter" << YAML::Value << params_config_.field_filter;
  yaml_out << YAML::Key << "field_h_min" << YAML::Value << params_config_.goalpost_threshold.h_min;
  yaml_out << YAML::Key << "field_h_max" << YAML::Value << params_config_.goalpost_threshold.h_max;
  yaml_out << YAML::Key << "field_s_min" << YAML::Value << params_config_.goalpost_threshold.s_min;
  yaml_out << YAML::Key << "field_s_max" << YAML::Value << params_config_.goalpost_threshold.s_max;
  yaml_out << YAML::Key << "field_v_min" << YAML::Value << params_config_.goalpost_threshold.v_min;
  yaml_out << YAML::Key << "field_v_max" << YAML::Value << params_config_.goalpost_threshold.v_max;
  yaml_out << YAML::Key << "ellipse_size" << YAML::Value << params_config_.ellipse_size;
  yaml_out << YAML::Key << "gain" << YAML::Value << params_config_.gain;
  yaml_out << YAML::Key << "brightness" << YAML::Value << params_config_.brightness;
  yaml_out << YAML::Key << "exposure" << YAML::Value << params_config_.exposure;
  yaml_out << YAML::Key << "filter_debug" << YAML::Value << params_config_.debug;
  yaml_out << YAML::EndMap;

  // output to file
  std::ofstream fout(param_path_.c_str());
  fout << yaml_out.c_str();
}

void GoalpostDetector::publishConfig()
{
  op3_camera_setting_tool::V4lParameters params;

  op3_camera_setting_tool::V4lParameter param_gain;
  param_gain.name = "gain";
  param_gain.value = params_config_.gain;

  op3_camera_setting_tool::V4lParameter param_brightness;
  param_brightness.name = "brightness";
  param_brightness.value = params_config_.brightness;

  op3_camera_setting_tool::V4lParameter param_exposure;
  param_exposure.name = "exposure_absolute";
  param_exposure.value = params_config_.exposure;

  params.video_parameter.push_back(param_gain);
  params.video_parameter.push_back(param_brightness);
  params.video_parameter.push_back(param_exposure);

  camera_params_pub_.publish(params);
}

void GoalpostDetector::setInputImage(const cv::Mat &inIm)
{
  in_image_ = inIm.clone();
  ori_image_ = inIm.clone();

  if (params_config_.debug == false)
    out_image_ = in_image_.clone();
}

void GoalpostDetector::getOutputImage(cv::Mat &outIm)
{
  drawOutputImage();
  outIm = out_image_.clone();
}

void GoalpostDetector::drawOutputImage()
{
  if (params_config_.debug == true)
    out_image_ = in_image_.clone();

  ///////for HoughLines_Vertical///////
  for(size_t i = 0; i < filtered_lines_v.size(); i++ )
  {
    cv::Point pt_vertical_1, pt_vertical_2;
    getEndpoints(filtered_lines_v[i][0], filtered_lines_v[i][1], pt_vertical_1, pt_vertical_2, Vertical);
    cv::line(out_image_, pt_vertical_1, pt_vertical_2, cv::Scalar(0,255,0), 2);

    /////Mean for Vertical Lines
    if(post_n == 1)
    {
      cv::line(out_image_, pt_vertical_mean_top, pt_vertical_mean_bottom, cv::Scalar(255,0,0), 3);
      cv::drawMarker(out_image_, pt_top_unknown, (254,254,254), 6, 20, 2);
    }
    if(post_n == 2)
    {
      //left_post
      cv::line(out_image_, pt_vertical_left_top, pt_vertical_left_bottom, cv::Scalar(255,0,127), 3);
      cv::drawMarker(out_image_, pt_top_left, (254,254,254), 6, 20, 2);

      //right_post
      cv::line(out_image_, pt_vertical_right_top, pt_vertical_right_bottom, cv::Scalar(255,0,127), 3);
      cv::drawMarker(out_image_, pt_top_right, (254,254,254), 6, 20, 2);
    }
  }

  ///////for HoughLines_Horizontal///////
  for(size_t i = 0; i < filtered_lines_h.size(); i++ )
  {
    cv::Point pt_horizontal_1, pt_horizontal_2;
    getEndpoints(filtered_lines_h[i][0], filtered_lines_h[i][1], pt_horizontal_1, pt_horizontal_2, Horizontal);
    cv::line(out_image_, pt_horizontal_1, pt_horizontal_2, cv::Scalar(0,255,0), 2);

    /////Mean for Horizontal Line/////
    cv::line(out_image_, pt_horizontal_leftend, pt_horizontal_rightend, cv::Scalar(255,0,127), 3);
  }


  ///////for HoughLinesP///////
  //  for(size_t i = 0; i < filtered_lines.size(); i++ )
  //  {
  //    cv::Vec4f l = filtered_lines[i];
  //    cv::line(out_image_, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255,0,0), 2);
  //  }
}

void GoalpostDetector::getEndpoints(const float rho, const float theta, cv::Point &pt1, cv::Point &pt2, int type)
{
  //1 pixel apart from the most end point to check the around pixels.
  if(type == Vertical)
  {
    //Top Point
    pt1 = cv::Point( cvRound( rho / cos(theta) ), cvRound(1) );
    //Bottom Point
    pt2 = cv::Point( cvRound((rho - in_image_.rows * sin(theta)) / cos(theta)),
                     cvRound(in_image_.rows - 1) );
  }
  else if(type == Horizontal)
  {
    //Left Point
    pt1 = cv::Point( cvRound(1), cvRound(rho / sin(theta)) );
    //Right Point
    pt2 = cv::Point( cvRound(in_image_.cols - 1),
                     cvRound((rho - in_image_.cols * cos(theta)) / sin(theta)) );
  }
}

void GoalpostDetector::filterImage()
{
  if (!in_image_.data)
    return;

  cv::Mat img_hsv, img_post_filtered;
  //RGB to HSV Conversion
  cv::cvtColor(in_image_, img_hsv, cv::COLOR_RGB2HSV);

  //HSV to Grey image Conversion using param values
  inRangeHsv(img_hsv, params_config_.goalpost_threshold, img_post_filtered);

  // morphology : open and close
  morphology(img_post_filtered, img_post_filtered, params_config_.ellipse_size);

  //Gray to RGB Conversion
  cv::cvtColor(img_post_filtered, in_image_, cv::COLOR_GRAY2RGB);

  ///////////////EDITING///////////////
  if (params_config_.field_filter == true)
  {
    // mask
    // cv::Mat img_mask;

    // check hsv range
    inRangeHsv(img_hsv, params_config_.field_threshold, img_field_filtered);
    morphology(img_field_filtered, img_field_filtered, params_config_.ellipse_size);

    for(size_t i = 1; i < img_field_filtered.rows; i++)
    {
      for(size_t j = 1; j < img_field_filtered.cols; j++)
      {
        if (img_field_filtered.at<uchar>(i,j) == 255 && img_post_filtered.at<uchar>(i,j) != 255)
          img_post_filtered.at<uchar>(i,j) = 150;
      }
    }
    //Gray to RGB Conversion
    cv::cvtColor(img_post_filtered, img_field_filtered, cv::COLOR_GRAY2RGB);
  }
  /////////////////////////////////////
}

void GoalpostDetector::inRangeHsv(const cv::Mat &input_img, const HsvFilter &filter_value, cv::Mat &output_img)
{
  // 0-360 -> 0-180 for hue value
  int scaled_hue_min = static_cast<int>(filter_value.h_min *0.5);
  int scaled_hue_max = static_cast<int>(filter_value.h_max *0.5);

  if (scaled_hue_min <= scaled_hue_max)
  {
    cv::Scalar min_value = cv::Scalar(scaled_hue_min, filter_value.s_min, filter_value.v_min, 0);
    cv::Scalar max_value = cv::Scalar(scaled_hue_max, filter_value.s_max, filter_value.v_max, 0);

    //HSV to Grey image(CV_8U) Convertion
    cv::inRange(input_img, min_value, max_value, output_img);
  }

  else
  {
    cv::Mat lower_hue_range, upper_hue_range;
    cv::Scalar min_value, max_value;

    //0~max_value
    min_value = cv::Scalar(0, filter_value.s_min, filter_value.v_min, 0);
    max_value = cv::Scalar(scaled_hue_max, filter_value.s_max, filter_value.v_max, 0);
    cv::inRange(input_img, min_value, max_value, lower_hue_range);

    //min_value~179
    min_value = cv::Scalar(scaled_hue_min, filter_value.s_min, filter_value.v_min, 0);
    max_value = cv::Scalar(179, filter_value.s_max, filter_value.v_max, 0);
    cv::inRange(input_img, min_value, max_value, upper_hue_range);

    cv::bitwise_or(lower_hue_range, upper_hue_range, output_img);
  }
}

void GoalpostDetector::morphology(const cv::Mat &intput_img, cv::Mat &output_img, int ellipse_size)
{
  /*
  cv::morphologyEx(intput_img, output_img,cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ellipse_size, ellipse_size));
  cv::morphologyEx(intput_img, output_img,cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ellipse_size, ellipse_size));
  */

  cv::erode(intput_img, output_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ellipse_size, ellipse_size)));
  cv::dilate(output_img, output_img,
             cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ellipse_size * 2, ellipse_size * 2)));

  cv::dilate(output_img, output_img,
             cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ellipse_size, ellipse_size)));
  cv::erode(output_img, output_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ellipse_size, ellipse_size)));

  //Remove Noise
  for(size_t i = 1; i < output_img.rows; i++)
  {
    for(size_t j = 1; j < output_img.cols; j++)
    {
      if (output_img.at<uchar>(i,j) == 0 &&
          output_img.at<uchar>(i-1,j-1)+output_img.at<uchar>(i-1,j)+output_img.at<uchar>(i-1,j+1)+output_img.at<uchar>(i,j-1)+output_img.at<uchar>(i,j+1)+
          output_img.at<uchar>(i+1,j-1)+output_img.at<uchar>(i+1,j)+output_img.at<uchar>(i+1,j+1) > 255*6)
        output_img.at<uchar>(i,j) = 255;
      if (output_img.at<uchar>(i,j) == 255 &&
          output_img.at<uchar>(i-1,j-1)+output_img.at<uchar>(i-1,j)+output_img.at<uchar>(i-1,j+1)+output_img.at<uchar>(i,j-1)+output_img.at<uchar>(i,j+1)+
          output_img.at<uchar>(i+1,j-1)+output_img.at<uchar>(i+1,j)+output_img.at<uchar>(i+1,j+1) < 255*4)
        output_img.at<uchar>(i,j) = 0;
    }
  }
}

void GoalpostDetector::houghDetection(const unsigned int imgEncoding)
{
  cv::Mat gray_image;
  lines_.clear();

  // Because the Image type is RGB after "filterImage"
  if (imgEncoding == IMG_RGB8)
    cv::cvtColor(in_image_, gray_image, CV_RGB2GRAY);

  //Reduce the noise so we avoid false detection
  cv::GaussianBlur(gray_image, gray_image, cv::Size(params_config_.gaussian_blur_size, params_config_.gaussian_blur_size),
                   params_config_.gaussian_blur_sigma);

  //Canny Edge Detection
  cv::Canny(gray_image, gray_image, params_config_.canny_th_low, params_config_.canny_th_high, 3);
  //    params_config_.hough_accum_th : params_config_.hough_accum_th * 0.5;
  double hough_accum_th = params_config_.hough_accum_th;

  //Apply the Hough Transform to find the lines
  //cv::fitLine(gray_image, lines_, CV_DIST_L2, 0, 0.01, 0.01);
  //cv::HoughLinesP(gray_image, lines_, RHO, THETA * M_PI/180, hough_accum_th, params_config_.min_line_length, params_config_.max_line_gap);
  cv::HoughLines(gray_image, lines_, RHO, THETA*M_PI/180, hough_accum_th);
  setSlopeDegree();
}

void GoalpostDetector::setSlopeDegree()
{
  double slope_degree = 0.0;
  filtered_lines_v.clear();
  filtered_lines_h.clear();

  ///////for HoughLines///////
  for (size_t i = 0; i < lines_.size(); i++)
  {
    slope_degree = lines_[i][1] * 180 / M_PI;
    //printf("%f, %f \n", lines_[i][0], lines_[i][1]*180/M_PI);

    /////for Vertical Lines/////
    if (slope_degree < 20 || 160 < slope_degree)
    {
      filtered_lines_v.push_back(lines_[i]);
      filtered_lines_.push_back(lines_[i]);
    }
    /////for Horizontal Lines/////
    if (70 < slope_degree && slope_degree < 110)
    {
      filtered_lines_h.push_back(lines_[i]);
      filtered_lines_.push_back(lines_[i]);
    }
  }

  ///////for HoughLinesP///////
  //  for(size_t i = 0; i < lines_.size(); i++ )
  //  {
  //    slope_degree = abs(atan2((lines_[i][0] - lines_[i][2]), (lines_[i][1] - lines_[i][3])) * 180 / M_PI);

  //    if (170 < slope_degree && slope_degree < 190)
  //      filtered_lines.push_back(lines_[i]);
  //  }
getMeanline();
}

void GoalpostDetector::getMeanline()
{
  float rho_sum_h, theta_sum_h, rho_sum_v, theta_sum_v = 0.0;
  float rho_sum_vl, theta_sum_vl, rho_sum_vr, theta_sum_vr = 0.0;
  float rho_max_v, rho_min_v = 0.0;

  rho_mean_h, theta_mean_h = 0.0;
  rho_mean_v, theta_mean_v = 0.0;
  rho_mean_vl, theta_mean_vl, rho_mean_vr, theta_mean_vr = 0.0;

  post_n = 0;
  cross_bar = false;

  //Horizontal Lines' Mean Value
  if (filtered_lines_h.size() > 0)
  {
    cross_bar = true;
    for(size_t i = 0; i < filtered_lines_h.size(); i++)
    {
      rho_sum_h += filtered_lines_h[i][0];
      theta_sum_h += filtered_lines_h[i][1];
    }
    rho_mean_h = rho_sum_h / filtered_lines_h.size();
    theta_mean_h = theta_sum_h / filtered_lines_h.size();
  }

  //Vertical Lines' Mean Value
  if (filtered_lines_v.size() > 0)
  {
    post_n = 1;
    //Mean for all Vertical values
    for(size_t i = 0; i < filtered_lines_v.size(); i++)
    {
      rho_sum_v += abs(filtered_lines_v[i][0]);
      if (filtered_lines_v[i][1] > 0.5*M_PI)
        theta_sum_v += filtered_lines_v[i][1] - 0.5*M_PI;
      else
        theta_sum_v += filtered_lines_v[i][1] + 0.5*M_PI;

      if(abs(filtered_lines_v[i][0]) <= rho_min_v)
        rho_min_v = filtered_lines_v[i][0];
      if(abs(filtered_lines_v[i][0]) > rho_max_v)
        rho_max_v = filtered_lines_v[i][0];
    }
    rho_mean_v = rho_sum_v / filtered_lines_v.size();
    theta_mean_v = theta_sum_v / filtered_lines_v.size();
    if (theta_mean_v < 0.5*M_PI)
    {
      theta_mean_v = theta_mean_v + 0.5*M_PI;
      rho_mean_v = -rho_mean_v;
    }
    else
      theta_mean_v = theta_mean_v - 0.5*M_PI;

    //Check the number of detected posts with pixel distance
    //if detected Posts are 2,
    if (abs(rho_mean_v - rho_max_v) > 200 || abs(rho_mean_v - rho_min_v) > 200)
    {
      post_n = 2;
      std::vector<cv::Vec2f> left_post_;
      std::vector<cv::Vec2f> right_post_;

      for(size_t i = 0; i < filtered_lines_v.size(); i++)
      {
        if (abs(rho_mean_v) >= abs(filtered_lines_v[i][0]))
          left_post_.push_back(filtered_lines_v[i]);
        else
          right_post_.push_back(filtered_lines_v[i]);
      }

      /////left post/////
      for(size_t i = 0; i < left_post_.size(); i++)
      {
        rho_sum_vl += abs(left_post_[i][0]);
        if(left_post_[i][1] > 0.5*M_PI)
          theta_sum_vl += left_post_[i][1] - 0.5*M_PI;
        else
          theta_sum_vl += left_post_[i][1] + 0.5*M_PI;
      }
      rho_mean_vl = rho_sum_vl / left_post_.size();
      theta_mean_vl = theta_sum_vl / left_post_.size();
      if (theta_mean_vl < 0.5*M_PI)
      {
        theta_mean_vl = theta_mean_vl + 0.5*M_PI;
        rho_mean_vl = -rho_mean_vl;
      }
      else
        theta_mean_vl = theta_mean_vl - 0.5*M_PI;

      /////right post/////
      for(size_t i = 0; i < right_post_.size(); i++)
      {
        rho_sum_vr += abs(right_post_[i][0]);
        if(right_post_[i][1] > 0.5*M_PI)
          theta_sum_vr += right_post_[i][1] - 0.5*M_PI;
        else
          theta_sum_vr += right_post_[i][1] + 0.5*M_PI;
      }
      rho_mean_vr = rho_sum_vr / right_post_.size();
      theta_mean_vr = theta_sum_vr / right_post_.size();
      if (theta_mean_vr < 0.5*M_PI)
      {
        theta_mean_vr = theta_mean_vr + 0.5*M_PI;
        rho_mean_vr = -rho_mean_vr;
      }
      else
        theta_mean_vr = theta_mean_vr - 0.5*M_PI;
    }
  }
}

bool GoalpostDetector::getKeyPoints()
{
  if(cross_bar == true)
    getEndpoints(rho_mean_h, theta_mean_h, pt_horizontal_leftend, pt_horizontal_rightend, Horizontal);

  switch(post_n)
  {

  case 0:
    return false;
    break;
  case 1:
  {
    getEndpoints(rho_mean_v, theta_mean_v, pt_vertical_mean_top, pt_vertical_mean_bottom, Vertical);
    if(cross_bar == true)
    {
      getIntersectionPoint(pt_horizontal_leftend, pt_horizontal_rightend, pt_vertical_mean_top, pt_vertical_mean_bottom, pt_top_unknown);
    }
  }

  if (post_n == 2)
  {
    //get Top Points
    getEndpoints(rho_mean_vl, theta_mean_vl, pt_vertical_left_top, pt_vertical_left_bottom, Vertical);
    getEndpoints(rho_mean_vr, theta_mean_vr, pt_vertical_right_top, pt_vertical_right_bottom, Vertical);
    getIntersectionPoint(pt_horizontal_leftend, pt_horizontal_rightend, pt_vertical_left_top, pt_vertical_left_bottom, pt_top_left);
    getIntersectionPoint(pt_horizontal_leftend, pt_horizontal_rightend, pt_vertical_right_top, pt_vertical_right_bottom, pt_top_right);

    //get Bottom Points
    cv::Mat hsv_image_;
    cv::cvtColor(ori_image_, hsv_image_, CV_RGB2HSV);
    cv::LineIterator it(hsv_image_, pt_vertical_left_top, pt_vertical_left_bottom, 8);
    std::vector<cv::Vec3b> buf(it.count);
    std::vector<cv::Point> points(it.count);

    //for(int i = 0; i < it.count; i++, ++it)
    //{
    //    buf[i] = *(const cv::Vec3b)*it;

    for(int i = 0; i < it.count; i++)
    {
        buf.push_back( cv::Vec3b(*it) );
        ++it;
        points[i] = it.pos();\
    }
  }
  }

  return true;
}

bool GoalpostDetector::getIntersectionPoint(const cv::Point &a1, const cv::Point &a2, const cv::Point &b1, const cv::Point &b2, cv::Point &intPnt)
{
  cv::Point r(a2-a1);
  cv::Point s(b2-b1);

  if(cross(r,s) == 0)
    return false;

  double t = cross(b1-a1,s)/cross(r,s);

  intPnt = a1 + t*r;
  return true;
}

double GoalpostDetector::cross(const cv::Point& v1,const cv::Point& v2)
{
  return v1.x*v2.y - v1.y*v2.x;
}

bool GoalpostDetector::checkNearbyPixels(const cv::Point& pt, const cv::Mat &input_img)
{
  int count = 0;
  for(size_t i =  pt.x-1; i < pt.x+1; i++)
  {
    for(size_t j = pt.y-1; j < pt.y+1; j++)
    {
      if (input_img.at<cv::Vec3b>(i,j)[0] == 150)
        count++;
      else
        count--;
    }
  }
  if (count > 0)
    return true;
  else
    return false;
}

}   // namespace robotis_op
