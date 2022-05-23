/*****************************************************************************
 *                                                                            *
 *  Copyright (C) 2021 Steven Sun.                                        *
 *                                                                            *
 *  Licensed under the Apache License, Version 2.0 (the "License");           *
 *  you may not use this file except in compliance with the License.          *
 *  You may obtain a copy of the License at                                   *
 *                                                                            *
 *      http://www.apache.org/licenses/LICENSE-2.0                            *
 *                                                                            *
 *  Unless required by applicable law or agreed to in writing, software       *
 *  distributed under the License is distributed on an "AS IS" BASIS,         *
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
 *  See the License for the specific language governing permissions and       *
 *  limitations under the License.                                            *
 *                                                                            *
 *****************************************************************************/
#define LINUX_X86
#ifndef LINUX_X86
#define LINUX_ARM
#endif

// for common
#include <iostream>
#include <thread>
// for camera driver
#include <oni_camera.h>
#include <uvc_camera.h>

// for pangolin visualization
#ifdef LINUX_X86
#include <viewer.h>
#endif

// for opencv
#include <opencv4/opencv2/opencv.hpp>

// for tflite
#include "common_helper_cv.h"
#include "image_processor.h"

// for orb
#include "src/orb/ORBextractor.h"

/*** Macro ***/
#define WORK_DIR RESOURCE_DIR
#define DEFAULT_INPUT_IMAGE RESOURCE_DIR "/kite.jpg"
#define LOOP_NUM_FOR_TIME_MEASUREMENT 10

#define NFEATURES 1000
#define NTHRESH 20


void do_something(cv::Mat &input_img, std::vector<cv::KeyPoint> &key_points, int threshold);

// Input camera bus number to choose a specific camera. You can use "lsusb" to check which bus your camera is on.
// https://www.cnblogs.com/avril/archive/2010/03/22/1691477.html
int main(int argc, char** argv) {
  printf("Orbbec camera driver!\n");
  printf("Chooseing camera on bus:%s\n", argv[1]);
  // RGB camera using UVC
  UVCCamera uvc_camera;
  uvc_camera.setParams();
  uvc_camera.openCamera();
  // Depth camera using OpneNI2
  OniCamera oni_camera;
  oni_camera.camera_loc_ = *argv[1];
  oni_camera.depth_uri_str_ = oni_camera.enumerateDevices();
  oni_camera.openCamera();
  oni_camera.seOnitLDP(false);

// #ifdef LINUX_X86
//   // Viewer for point cloud
//   viewer::Viewer main_viewer(480, 640, "pcl");
//   main_viewer.setInputCloud(oni_camera.point_cloud_);
//   std::thread display_loop;
//   display_loop = std::thread(&viewer::Viewer::run, &main_viewer);
// #endif

  // /* Initialize image processor library */
  // ImageProcessor::InputParam input_param = {WORK_DIR, 4};
  // if (ImageProcessor::Initialize(input_param) != 0) {
  //   printf("Initialization Error\n");
  // }

  /* Initialize orb extractor */

  ORB_SLAM3::ORBextractor extractor(NFEATURES, NTHRESH);

  /*** Process for each frame ***/
  int32_t frame_cnt = 0;
  cv::namedWindow("raw_depth", 0);
  cv::namedWindow("rgb_img", 0);
  while (1) {
    // DEPTH
    oni_camera.GetOniStreamData();
    if (oni_camera.oni_depth_frame_.isValid()) {
      openni::DepthPixel* pDepth = (openni::DepthPixel*)oni_camera.oni_depth_frame_.getData();
      cv::Mat raw_depth(ONI_HEIGHT, ONI_WIDTH, CV_16UC1, pDepth);
      cv::flip(raw_depth, raw_depth, 1);
      cv::Mat tmp_depth;
      // Show raw_depth
      raw_depth.convertTo(tmp_depth, CV_8UC1, 1. / 2.);
      if (!tmp_depth.empty()) {
        cv::imshow("raw_depth", tmp_depth);
        cv::waitKey(1);
      }
      // oni_camera.generatePointCloud(uvc_camera.raw_rgb_);
    }

    // RGB
    cv::Mat rgb_img = uvc_camera.getImage();
    if (!rgb_img.empty()) {
      const auto& start_time = std::chrono::steady_clock::now();
      // 测试函数
      cv::Mat mImGray;
      cv::cvtColor(rgb_img,mImGray,cv::COLOR_BGR2GRAY);
      std::vector<cv::KeyPoint> key_points;
      cv::Mat descriptors;
      int thresh = 20;
      extractor(mImGray, cv::Mat(), key_points, descriptors);
      printf("get %d keypoints.\n", key_points.size());
      // do_something(mImGray, key_points, thresh);
      const auto& end_time = std::chrono::steady_clock::now();
      double time_image_process = (end_time - start_time).count() / 1000000.0;
      // 把keypoint画出来
      for(int i = 0; i < key_points.size(); i ++){
        cv::circle(rgb_img,cv::Point(key_points[i].pt.x,key_points[i].pt.y),3,CV_RGB(0,255,0),1);
      }
      cv::imshow("rgb_img", rgb_img);
      cv::waitKey(1);
    }

    usleep(100);
  }
  return 0;
}

void do_something(cv::Mat &input_img, std::vector<cv::KeyPoint> &key_points, int threshold){
  printf("Extracting FAST features...\n");
  FAST(input_img,key_points,threshold,true);  
  printf("Got %d FAST feature.\n", key_points.size());                                                         
}