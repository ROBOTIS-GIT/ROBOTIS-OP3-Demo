#ifndef GOALPOST_DETECTOR_CONFIG_H
#define GOALPOST_DETECTOR_CONFIG_H




namespace robotis_op
{

//constants
const int GAUSSIAN_BLUR_SIZE_DEFAULT = 7;
const double GAUSSIAN_BLUR_SIGMA_DEFAULT = 2;
const double CANNY_TH_HIGH_DEFAULT = 200;
const double CANNY_TH_LOW_DEFAULT = 70;
const double HOUGH_ACCUM_TH_DEFAULT = 120;
const int MIN_LENGTH_DEFAULT = 20;
const int MAX_GAP_DEFAULT = 10;
const unsigned int IMG_MONO = 0;
const unsigned int IMG_RGB8 = 1;
const int FILTER_RANGE_DEFAULT_MIN = 160;
const int FILTER_RANGE_DEFAULT_MAX = 255;
const int FILTER_H_MIN_DEFAULT = 0;
const int FILTER_H_MAX_DEFAULT = 30;
const int FILTER_S_MIN_DEFAULT = 0;
const int FILTER_S_MAX_DEFAULT = 255;
const int FILTER_V_MIN_DEFAULT = 0;
const int FILTER_V_MAX_DEFAULT = 255;
const int GAIN = 255;
const int BRIGHTNESS = 80;
const int EXPOSURE = 120;
const int ELLIPSE_SIZE = 5;

class HsvFilter
{
public:
  HsvFilter()
    : h_min(FILTER_H_MIN_DEFAULT),
      h_max(FILTER_H_MAX_DEFAULT),
      s_min(FILTER_S_MIN_DEFAULT),
      s_max(FILTER_S_MAX_DEFAULT),
      v_min(FILTER_V_MIN_DEFAULT),
      v_max(FILTER_V_MAX_DEFAULT)

  {
  }
  int h_min;
  int h_max;
  int s_min;
  int s_max;
  int v_min;
  int v_max;
};

class PostDetectorConfig
{
public:
  int gaussian_blur_size;         // size of gaussian blur kernel mask [pixel]
  double gaussian_blur_sigma;     // sigma of gaussian blur kernel mask [pixel]
  double canny_th_low;            // threshold of the edge detector.
  double canny_th_high;           // threshold of the edge detector.
  double hough_accum_th;          // accumulator threshold to decide circle detection
  double max_line_gap;            // Maximum line gap between lines
  double min_line_length;         // minimum line length allowed
  HsvFilter goalpost_threshold;   // goalpost filter threshold
  HsvFilter field_threshold;      // field filter threshold
  int ellipse_size;
  int gain;
  int brightness;
  int exposure;
  bool field_filter;
  bool debug;                     // to debug log

  PostDetectorConfig()
    : canny_th_low(CANNY_TH_LOW_DEFAULT),
      canny_th_high(CANNY_TH_HIGH_DEFAULT),
      hough_accum_th(HOUGH_ACCUM_TH_DEFAULT),
      max_line_gap(MAX_GAP_DEFAULT),
      min_line_length(MIN_LENGTH_DEFAULT),
      goalpost_threshold(),
      ellipse_size(ELLIPSE_SIZE),
      gain(GAIN),
      brightness(BRIGHTNESS),
      exposure(EXPOSURE),
      field_filter(true),
      debug(false)
  {
  }

  ~PostDetectorConfig()
  {
  }
};
}

#endif // GOALPOST_DETECTOR_CONFIG_H
