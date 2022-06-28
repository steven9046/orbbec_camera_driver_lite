/*****************************************************************************
 *                                                                            *
 *  Copyright (C) 2022 Steven Sun.                                        *
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

// for common
#include <iostream>
// for camera driver
#include <oni_camera.h>
#include <uvc_camera.h>
// for opencv
#include <opencv4/opencv2/opencv.hpp>

using namespace cv;

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
  oni_camera.getCameraParams();

  Mat imDepth;
  namedWindow("raw_depth", 0);
  namedWindow("rgb_img", 1);

  Matx<float, 2, 2> a = {2,4,5,6};
  Matx<float, 2, 2> b = {2,2,2,2};
  auto c = a.div(b);
  std::cout << c(0,0) << " " << c(0,1) << std::endl;
  std::cout << c(1,0) << " " << c(1,1) << std::endl;

  // std::cout << c(0,0) << " " << c(0,1) << " " << c(0,2) << std::endl;
  // std::cout << c(1,0) << " " << c(1,1) << " " << c(1,2) << std::endl;
  // std::cout << c(8) << " " << c(9) << c(10) << " " << c(11) << std::endl;
  // std::cout << c(12) << " " << c(13) << c(14) << " " << c(15) << std::endl;


  // std::cout << c(0,0) << " " << c(0,1) << std::endl;
  // std::cout << c(1,0) << " " << c(1,1) << std::endl;
  // std::cout << f(0,0) << " " << f(0,1) << std::endl;
  // std::cout << f(1,0) << " " << f(1,1) << std::endl;
  
  // std::cout << g(0,0) << " " << g(0,1) << std::endl;
  // std::cout << g(1,0) << " " << g(1,1) << std::endl;
  // std::cout << b[0] << " " << b[1] << " " << b[2] << std::endl;
  // auto c = a.mul(b);
  // std::cout << c[0] << " " << c[1] << " " << c[2] << std::endl;
  // auto d = a.cross(b);
  // std::cout << d[0] << " " << d[1] << " " << d[2] << std::endl;

  while (1) {
    // DEPTH
    oni_camera.GetOniStreamData();
    if (oni_camera.oni_depth_frame_.isValid()) {
      openni::DepthPixel* pDepth = (openni::DepthPixel*)oni_camera.oni_depth_frame_.getData();
      Mat raw_depth(ONI_HEIGHT, ONI_WIDTH, CV_16UC1, pDepth);
      // Show depth
      oni_camera.computeConvertedDepth(raw_depth, imDepth);
      if (!imDepth.empty()) {
        imshow("raw_depth", imDepth);
        waitKey(1);
      }
    }

    // RGB
    Mat rgb_img = uvc_camera.getImage();
    Mat image_for_show = rgb_img.clone();
    int height = rgb_img.rows;
    int width = rgb_img.cols;
    if (!rgb_img.empty()) {
      


      imshow("rgb_img", image_for_show);
      waitKey(1);

    }
    
    usleep(100);
  }

  return 0;
}
