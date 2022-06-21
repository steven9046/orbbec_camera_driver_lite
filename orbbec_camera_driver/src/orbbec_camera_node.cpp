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
// for flatbaffer message
#include "camera_generated.h"
// for zmq_publisher
#include "zmq_publisher.hpp"

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
  oni_camera.getCameraParams();

  cv::Mat imDepth;
  cv::namedWindow("raw_depth", 0);
  cv::namedWindow("rgb_img", 0);

  // 消息发布
  zmq_communicater::ZMQPublisher pub_;
    pub_.read_zmq_topic_to_port_file();
    std::string topic_name = "topic_1";
    pub_.init_publisher(topic_name);
  // 创建消息并序列化
  flatbuffers::FlatBufferBuilder builder;
  // auto Camera = new Message::Camera; // 默认构造函数被删除了
  // 创建内部格式数据
  auto name = builder.CreateString("Orbbec");
  float fx = 454.343;
  float fy = 454.343;
  float cx = 327.461;
  float cy = 246.672;
  float k1 = 0.0552932;
  float k2 = -0.0753816;
  float k3 = 0.;
  // 创建对象
  auto camera = Message::CreateCamera(builder, name, fx, fy, cx, cy, k1, k2, k3);
  builder.Finish(camera);
  // 获取数据区指针与大小
  uint8_t* buf = builder.GetBufferPointer();
  int size = builder.GetSize();

  while (1) {
    // DEPTH
    oni_camera.GetOniStreamData();
    if (oni_camera.oni_depth_frame_.isValid()) {
      openni::DepthPixel* pDepth = (openni::DepthPixel*)oni_camera.oni_depth_frame_.getData();
      cv::Mat raw_depth(ONI_HEIGHT, ONI_WIDTH, CV_16UC1, pDepth);
      // Show depth
      oni_camera.computeConvertedDepth(raw_depth, imDepth);
      if (!imDepth.empty()) {
        cv::imshow("raw_depth", imDepth);
        cv::waitKey(1);
      }
    }

    // RGB
    cv::Mat rgb_img = uvc_camera.getImage();
    cv::Mat image_for_show = rgb_img.clone();
    if (!rgb_img.empty()) {
      cv::imshow("rgb_img", image_for_show);
      cv::waitKey(1);
    }
    
    zmq::message_t msg(buf, size);
    pub_.publish(msg);

    usleep(100);
  }

  return 0;
}
