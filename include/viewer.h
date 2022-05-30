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

#include <pangolin/pangolin.h> 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "orb/MapDrawer.h"
namespace viewer{

class Viewer {
public:    
    Viewer();
    ~Viewer();
    Viewer(int height, int width, std::string name, ORB_SLAM3::MapDrawer* pMapDrawer);
    void setup();
    void setWindowName(std::string name);
    void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);
    void run();
    std::string window_name_;
    int window_height_, window_width_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_;
    ORB_SLAM3::MapDrawer* mpMapDrawer;
};
}