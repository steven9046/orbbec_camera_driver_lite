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

#include <pthread.h>
#include <viewer.h>
#include "orb/MapDrawer.h"

namespace viewer {

Viewer::Viewer(int height, int width, std::string name, ORB_SLAM3::MapDrawer* pMapDrawer) : mpMapDrawer(pMapDrawer)
{
  window_height_ = height;
  window_width_ = width;
  window_name_ = name;
  setup();
}

Viewer::~Viewer() {}

void Viewer::setup() {
  // create a window and bind its context to the main thread
  pangolin::CreateWindowAndBind(window_name_, window_width_, window_height_);

  // enable depth
  glEnable(GL_DEPTH_TEST);

  // unset the current context from the main thread
  pangolin::GetBoundWindow()->RemoveCurrent();
  point_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
}

void Viewer::setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud) { point_cloud_ = input_cloud; }

void Viewer::setWindowName(std::string name) { window_name_ = name; }

void Viewer::run() {
  // fetch the context and bind it to this thread
  pangolin::BindToContext(window_name_);

  // we manually need to restore the properties of the context
  glEnable(GL_DEPTH_TEST);

  // Define Projection and initial ModelView matrix
  pangolin::OpenGlRenderState s_cam(pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 100),
                                    pangolin::ModelViewLookAt(0, 0, -2, 0, 0, 0, pangolin::AxisNegY));

  // Create Interactive View in window
  pangolin::Handler3D handler(s_cam);
  pangolin::View& d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f).SetHandler(&handler);

  // glClear(GL_COLOR_BUFFER_BIT );
  // glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

  // pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.2,0.0);
  // pangolin::Var<double> plan_x("menu.plan_x",0);
  // pangolin::Var<double> plan_y("menu.plan_y",0);
  // pangolin::Var<double> now_yaw("menu.now_yaw",0);
  // pangolin::Var<double> plan_yaw("menu.plan_yaw",0);
  // pangolin::Var<double> now_plan_yaw("menu.now_plan_yaw",0);
    pangolin::OpenGlMatrix Twc, Twr;
    Twc.SetIdentity();
    pangolin::OpenGlMatrix Ow; // Oriented with g in the z axis
    Ow.SetIdentity();

  while (!pangolin::ShouldQuit()) {
    mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc,Ow);
    // Clear screen and activate view to render into
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    // s_cam.Follow(Twc);
    s_cam.Follow(Ow);
    // pangolin::glDrawColouredCube();  //画三维方块
    //线条长度
    pangolin::glDrawAxis(0.1);  //三维坐标轴，红——x轴，绿——y轴，蓝——z轴
    // // 画map
    // for (int i = 0; i < 64000; i++) {
    //   glColor3f(0.0f, 1.0f, 0.0f);
    //   glPointSize(1);
    //   glBegin(GL_POINTS);
    //   glVertex3d(i, i, i);
    //   glEnd();
    // }
    for (int i = 0; i < point_cloud_->points.size(); i++) {
      glColor3f(0.0f, 1.0f, 0.0f);
      glPointSize(1);
      glBegin(GL_POINTS);
      glVertex3d(point_cloud_->points[i].x, point_cloud_->points[i].y, point_cloud_->points[i].z);
      glEnd();
    }
    mpMapDrawer->DrawCurrentCamera(Twc);
    // Swap frames and Process Events
    pangolin::FinishFrame();
  }

  // unset the current context from the main thread
  pangolin::GetBoundWindow()->RemoveCurrent();
  // printf("testing class threads...\n");
}

}  // namespace viewer