/* Author: Seri Lee */

#ifndef GOALPOST_DETECTOR_H
#define GOALPOST_DETECTOR_H

#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//ros dependencies
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>

#include "goalpost_detector/Line.h"
#include "goalpost_detector/LineSetStamped.h"
#include "goalpost_detector/goalpost_detector_config.h"
#include "goalpost_detector/GoalpostDetectorParamsConfig.h"
#include "op3_camera_setting_tool/V4lParameters.h"

namespace robotis_op {

class GoalpostDetector
{
public:
  enum Line_Type
  {
    Vertical = 0,
    Horizontal = 1,
  };

  GoalpostDetector();
  ~GoalpostDetector();

  bool newImage();
  void process();
  void publishImage();
  void publishParams();
  void publishLines();


protected:
  const double RHO;
  const double THETA;

  void enableCallback(const std_msgs::Bool::ConstPtr &msg);
  void imageCallback(const sensor_msgs::ImageConstPtr & msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfo & msg);

  void dynParamCallback(goalpost_detector::GoalpostDetectorParamsConfig &config, uint32_t level);
  void printConfig();
  void saveConfig();
  void publishConfig();

  void setInputImage(const cv::Mat & inIm);
  void getOutputImage(cv::Mat & outIm);
  void drawOutputImage();
  void filterImage();
  void inRangeHsv(const cv::Mat &input_img, const HsvFilter &filter_value, cv::Mat &output_img);
  void morphology(const cv::Mat &intput_img, cv::Mat &output_img, int ellipse_size);
  void houghDetection(const unsigned int imgEncoding);
  void setSlopeDegree();
  void getMeanline();
  void getEndpoints(const float rho, const float theta, cv::Point &pt1, cv::Point &pt2, int type);
  bool getIntersectionPoint(const cv::Point &a1, const cv::Point &a2, const cv::Point &b1, const cv::Point &b2, cv::Point &intPnt);
  double cross(const cv::Point &v1, const cv::Point &v2);
  bool getKeyPoints();
  bool checkNearbyPixels(const cv::Point& pt, const cv::Mat &input_img);



  ros::NodeHandle nh_;
  ros::Subscriber enable_sub_;

  //image publisher/subscriber
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
  cv_bridge::CvImage cv_img_pub_;
  image_transport::Subscriber image_sub_;
  cv_bridge::CvImagePtr cv_img_ptr_sub_;

  //camera info publisher/subscriber
  sensor_msgs::CameraInfo camera_info_msg_;
  ros::Subscriber camera_info_sub_;
  ros::Publisher camera_info_pub_;
  ros::Publisher camera_params_pub_;

  //line set publisher
  goalpost_detector::LineSetStamped lines_msg_; //generated from the msg file
  ros::Publisher lines_pub_;

  cv::Mat in_image_;
  cv::Mat out_image_;
  cv::Mat ori_image_;
  cv::Mat img_field_filtered;
  std::vector<cv::Vec2f> lines_;
  std::vector<cv::Vec2f> filtered_lines_;
  std::vector<cv::Vec2f> filtered_lines_v;
  std::vector<cv::Vec2f> filtered_lines_h;

  float rho_mean_h, theta_mean_h, rho_mean_v, theta_mean_v;
  float rho_mean_vl, theta_mean_vl, rho_mean_vr, theta_mean_vr;
  int post_n;
  bool cross_bar;

  cv::Point pt_vertical_left_top, pt_vertical_left_bottom;
  cv::Point pt_vertical_right_top, pt_vertical_right_bottom;
  cv::Point pt_horizontal_leftend,pt_horizontal_rightend;
  cv::Point pt_vertical_mean_top,pt_vertical_mean_bottom;
  cv::Point pt_top_left, pt_top_right, pt_top_unknown;
  cv::Point pt_bottom_left, pt_bottom_right, pt_bottom_unknown;

  //image time stamp and frame id
  ros::Time sub_time_;
  std::string image_frame_id_;

  unsigned int img_encoding_;

  //dynamic reconfigure
  PostDetectorConfig params_config_;
  std::string param_path_;
  bool has_path_;

  dynamic_reconfigure::Server<goalpost_detector::GoalpostDetectorParamsConfig> param_server_;
  dynamic_reconfigure::Server<goalpost_detector::GoalpostDetectorParamsConfig>::CallbackType callback_fnc_;


  //flag indicating...
  bool enable_;
  bool new_image_flag_;
  bool init_param_;





};

}


#endif // GOALPOST_DETECTOR_H
