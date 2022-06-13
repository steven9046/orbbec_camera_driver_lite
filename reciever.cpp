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
// // for camera driver
// #include <oni_camera.h>
// #include <uvc_camera.h>

// // for pangolin visualization
// #ifdef LINUX_X86
// #include <viewer.h>
// #include "src/orb/MapDrawer.h"
// #endif

// for opencv
#include <opencv4/opencv2/opencv.hpp>

// // for tflite
// #include "common_helper_cv.h"
// #include "image_processor.h"

// // for orb
// #include "src/orb/ORBextractor.h"
// #include "src/orb/Frame.h"
// #include "src/orb/Tracking.h"
// #include "src/orb/Settings.h"
// #include "src/orb/MapDrawer.h"

// for zmq
#include "libzmq/zmq.hpp"
// for flatbuffers
#include "camera_generated.h"
#include <fstream>

/*** Macro ***/
#define WORK_DIR RESOURCE_DIR
#define DEFAULT_INPUT_IMAGE RESOURCE_DIR "/kite.jpg"
#define LOOP_NUM_FOR_TIME_MEASUREMENT 10

#define NFEATURES 1000
#define NTHRESH_INI 20  // 初次阈值
#define NTHRESH_MIN 7   // 最小阈值
#define NLEVELS 8
#define SCALEFACTOR 1.2

#define BF 0.04
#define THD 40.0

void zmq_msg_buffer_free(void* data, void* hint) { delete[] static_cast<uint8_t*>(data); }
// Input camera bus number to choose a specific camera. You can use "lsusb" to check which bus your camera is on.
// https://www.cnblogs.com/avril/archive/2010/03/22/1691477.html
int main(int argc, char** argv) {

  zmq::context_t ctx_(1); // 必须这样创建上下文才行
  // 创建套接字并且连接发布者
  std::unique_ptr<zmq::socket_t> sub_;
  sub_.reset(new zmq::socket_t(ctx_, ZMQ_SUB));
  // 这里还可以有许多设置，不知道是干什么的
  int conflate = 1;
  sub_->setsockopt(ZMQ_SUBSCRIBE, "", 0);
  sub_->setsockopt(ZMQ_CONFLATE, &conflate, sizeof(conflate)); //只接收一个消息？
  sub_->setsockopt(ZMQ_LINGER, 0);  // ALWAYS

  sub_->connect("tcp://127.0.0.1:5555");
  int zmq_i = 0;
  printf("going into mainloop...\n");
  while(1){
    zmq::message_t zmq_msg;
    sub_->recv(&zmq_msg);
    // zmqmessage 里的数据拿出来， .data() 返回的是个 void*, 这里转成char*
    auto data = zmq_msg.data();
    // 反序列化，这个默认传入参数是void*
    std::unique_ptr<Message::CameraT> cameraT = Message::UnPackCamera(data);

    std::cout << "deserialized name : " <<cameraT->name << std::endl;
    std::cout << "deserialized fx: " <<cameraT->fx << std::endl;
    std::cout << "deserialized fy: " <<cameraT->fy << std::endl;
    std::cout << "deserialized cx: " <<cameraT->cx << std::endl;
    std::cout << "deserialized cy: " <<cameraT->cy << std::endl;
    // auto data = (char*)zmq_msg.data();
    auto size = zmq_msg.size();
    // for(int i = 0; i < )
    // {
    // printf("Recv:%s\n", data[0]);
    // }
    // std::cout << "size: " << size << std::endl;
    // for(int i = 0; i < size; i ++){
    //   std::cout << data[i];
    // }
    printf("\n");

    printf("received message!\n");
  }

}