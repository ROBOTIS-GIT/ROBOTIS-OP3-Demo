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

#ifndef _DETECTOR_CONFIG_H_
#define _DETECTOR_CONFIG_H_

namespace robotis_op
{

//constants
const int GAUSSIAN_BLUR_SIZE_DEFAULT = 7;
const double GAUSSIAN_BLUR_SIGMA_DEFAULT = 2;
const double CANNY_EDGE_TH_DEFAULT = 130;
const double HOUGH_ACCUM_RESOLUTION_DEFAULT = 2;
const double MIN_CIRCLE_DIST_DEFAULT = 30;
const double HOUGH_ACCUM_TH_DEFAULT = 120;
const int MIN_RADIUS_DEFAULT = 30;
const int MAX_RADIUS_DEFAULT = 400;
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

class DetectorConfig
{
 public:
  int gaussian_blur_size;         // size of gaussian blur kernel mask [pixel]
  double gaussian_blur_sigma;     // sigma of gaussian blur kernel mask [pixel]
  double canny_edge_th;           // threshold of the edge detector.
  double hough_accum_resolution;  // resolution of the Hough accumulator, in terms of inverse ratio of image resolution
  double min_circle_dist;         // Minimum distance between circles
  double hough_accum_th;          // accumulator threshold to decide circle detection
  int min_radius;                 // minimum circle radius allowed
  int max_radius;                 // maximum circle radius allowed
  HsvFilter filter_threshold;     // filter threshold
  bool use_field;
  HsvFilter field_threshold;      // field filter threshold
  bool field_debug;
  bool use_second_filter;
  HsvFilter filter2_threshold;     // filter threshold
  int ellipse_size;
  bool debug;                     // to debug log

  DetectorConfig()
      : canny_edge_th(CANNY_EDGE_TH_DEFAULT),
        hough_accum_resolution(HOUGH_ACCUM_RESOLUTION_DEFAULT),
        min_circle_dist(MIN_CIRCLE_DIST_DEFAULT),
        hough_accum_th(HOUGH_ACCUM_TH_DEFAULT),
        min_radius(MIN_RADIUS_DEFAULT),
        max_radius(MAX_RADIUS_DEFAULT),
        filter_threshold(),
        use_field(false),
        field_threshold(),
        use_second_filter(false),
        filter2_threshold(),
        ellipse_size(ELLIPSE_SIZE),
        debug(false)
  {
  }

  ~DetectorConfig()
  {
  }
};

}
#endif  // _DETECTOR_CONFIG_H_
