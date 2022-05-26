/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include "Tracking.h"

// #include "ORBmatcher.h"
// #include "FrameDrawer.h"
// #include "Converter.h"
// #include "G2oTypes.h"
// #include "Optimizer.h"
// #include "Pinhole.h"
// #include "KannalaBrandt8.h"
// #include "MLPnPsolver.h"
// #include "GeometricTools.h"

#include <iostream>

// #include <mutex>
#include <chrono>

using namespace std;

namespace ORB_SLAM3
{

/**
 * @brief 构造函数， 这个构造函数没有干什么事情
 * @param[in] pSys          SLAM系统指针
 * @param[in] pMapDrawer    地图绘制器
 * @param[in] pViewer       可视化
 * @param[in] pAtlas        地图
 * @param[in] pKFDB         关键帧数据库 
 * 1. 加载配置文件，初始化一些相关数据结构
 */
Tracking::Tracking(Settings* settings):
    mState(NO_IMAGES_YET),          // 初始状态为无图像
    mTrackedFr(0),                  // 跟踪到的帧数
    mbStep(false),                  
    mbVO(false), 
    mbReadyToInitializate(false), 
    bStepByStep(false), 
    mnInitialFrameId(0), 
    mnFirstFrameId(0),
    mbOnlyTracking(false),
    mpCamera2(nullptr)
{
    // 1. 加载配置文件
    // Load camera parameters from settings file
    if(settings){
        newParameterLoader(settings);
    }
    // else{
    //     cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    //     bool b_parse_cam = ParseCamParamFile(fSettings);
    //     if(!b_parse_cam)
    //     {
    //         std::cout << "*Error with the camera parameters in the config file*" << std::endl;
    //     }

    //     // Load ORB parameters
    //     bool b_parse_orb = ParseORBParamFile(fSettings);
    //     if(!b_parse_orb)
    //     {
    //         std::cout << "*Error with the ORB parameters in the config file*" << std::endl;
    //     }

    //     bool b_parse_imu = true;
    //     if(sensor==System::IMU_MONOCULAR || sensor==System::IMU_STEREO || sensor==System::IMU_RGBD)
    //     {
    //         b_parse_imu = ParseIMUParamFile(fSettings);
    //         if(!b_parse_imu)
    //         {
    //             std::cout << "*Error with the IMU parameters in the config file*" << std::endl;
    //         }

    //         mnFramesToResetIMU = mMaxFrames;
    //     }

    //     if(!b_parse_cam || !b_parse_orb || !b_parse_imu)
    //     {
    //         std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
    //         try
    //         {
    //             throw -1;
    //         }
    //         catch(exception &e)
    //         {

    //         }
    //     }
    // }

    // initID = 0; lastID = 0;
    // mbInitWith3KFs = false;
    // mnNumDataset = 0;

    // // 这里可以配置多个相机？？ 输出相机信息
    // vector<GeometricCamera*> vpCams = mpAtlas->GetAllCameras();
    // std::cout << "There are " << vpCams.size() << " cameras in the atlas" << std::endl;
    // for(GeometricCamera* pCam : vpCams)
    // {
    //     std::cout << "Camera " << pCam->GetId();
    //     if(pCam->GetType() == GeometricCamera::CAM_PINHOLE)
    //     {
    //         std::cout << " is pinhole" << std::endl;
    //     }
    //     else if(pCam->GetType() == GeometricCamera::CAM_FISHEYE)
    //     {
    //         std::cout << " is fisheye" << std::endl;
    //     }
    //     else
    //     {
    //         std::cout << " is unknown" << std::endl;
    //     }
    // }

// #ifdef REGISTER_TIMES
//     vdRectStereo_ms.clear();
//     vdResizeImage_ms.clear();
//     vdORBExtract_ms.clear();
//     vdStereoMatch_ms.clear();
//     vdIMUInteg_ms.clear();
//     vdPosePred_ms.clear();
//     vdLMTrack_ms.clear();
//     vdNewKF_ms.clear();
//     vdTrackTotal_ms.clear();
// #endif
}

// #ifdef REGISTER_TIMES
// double calcAverage(vector<double> v_times)
// {
//     double accum = 0;
//     for(double value : v_times)
//     {
//         accum += value;
//     }

//     return accum / v_times.size();
// }

// double calcDeviation(vector<double> v_times, double average)
// {
//     double accum = 0;
//     for(double value : v_times)
//     {
//         accum += pow(value - average, 2);
//     }
//     return sqrt(accum / v_times.size());
// }

// double calcAverage(vector<int> v_values)
// {
//     double accum = 0;
//     int total = 0;
//     for(double value : v_values)
//     {
//         if(value == 0)
//             continue;
//         accum += value;
//         total++;
//     }

//     return accum / total;
// }

// double calcDeviation(vector<int> v_values, double average)
// {
//     double accum = 0;
//     int total = 0;
//     for(double value : v_values)
//     {
//         if(value == 0)
//             continue;
//         accum += pow(value - average, 2);
//         total++;
//     }
//     return sqrt(accum / total);
// }

// void Tracking::LocalMapStats2File()
// {
//     ofstream f;
//     f.open("LocalMapTimeStats.txt");
//     f << fixed << setprecision(6);
//     f << "#Stereo rect[ms], MP culling[ms], MP creation[ms], LBA[ms], KF culling[ms], Total[ms]" << endl;
//     for(int i=0; i<mpLocalMapper->vdLMTotal_ms.size(); ++i)
//     {
//         f << mpLocalMapper->vdKFInsert_ms[i] << "," << mpLocalMapper->vdMPCulling_ms[i] << ","
//           << mpLocalMapper->vdMPCreation_ms[i] << "," << mpLocalMapper->vdLBASync_ms[i] << ","
//           << mpLocalMapper->vdKFCullingSync_ms[i] <<  "," << mpLocalMapper->vdLMTotal_ms[i] << endl;
//     }

//     f.close();

//     f.open("LBA_Stats.txt");
//     f << fixed << setprecision(6);
//     f << "#LBA time[ms], KF opt[#], KF fixed[#], MP[#], Edges[#]" << endl;
//     for(int i=0; i<mpLocalMapper->vdLBASync_ms.size(); ++i)
//     {
//         f << mpLocalMapper->vdLBASync_ms[i] << "," << mpLocalMapper->vnLBA_KFopt[i] << ","
//           << mpLocalMapper->vnLBA_KFfixed[i] << "," << mpLocalMapper->vnLBA_MPs[i] << ","
//           << mpLocalMapper->vnLBA_edges[i] << endl;
//     }


//     f.close();
// }

// void Tracking::TrackStats2File()
// {
//     ofstream f;
//     f.open("SessionInfo.txt");
//     f << fixed;
//     f << "Number of KFs: " << mpAtlas->GetAllKeyFrames().size() << endl;
//     f << "Number of MPs: " << mpAtlas->GetAllMapPoints().size() << endl;

//     f << "OpenCV version: " << CV_VERSION << endl;

//     f.close();

//     f.open("TrackingTimeStats.txt");
//     f << fixed << setprecision(6);

//     f << "#Image Rect[ms], Image Resize[ms], ORB ext[ms], Stereo match[ms], IMU preint[ms], Pose pred[ms], LM track[ms], KF dec[ms], Total[ms]" << endl;

//     for(int i=0; i<vdTrackTotal_ms.size(); ++i)
//     {
//         double stereo_rect = 0.0;
//         if(!vdRectStereo_ms.empty())
//         {
//             stereo_rect = vdRectStereo_ms[i];
//         }

//         double resize_image = 0.0;
//         if(!vdResizeImage_ms.empty())
//         {
//             resize_image = vdResizeImage_ms[i];
//         }

//         double stereo_match = 0.0;
//         if(!vdStereoMatch_ms.empty())
//         {
//             stereo_match = vdStereoMatch_ms[i];
//         }

//         double imu_preint = 0.0;
//         if(!vdIMUInteg_ms.empty())
//         {
//             imu_preint = vdIMUInteg_ms[i];
//         }

//         f << stereo_rect << "," << resize_image << "," << vdORBExtract_ms[i] << "," << stereo_match << "," << imu_preint << ","
//           << vdPosePred_ms[i] <<  "," << vdLMTrack_ms[i] << "," << vdNewKF_ms[i] << "," << vdTrackTotal_ms[i] << endl;
//     }

//     f.close();
// }

// void Tracking::PrintTimeStats()
// {
//     // Save data in files
//     TrackStats2File();
//     LocalMapStats2File();


//     ofstream f;
//     f.open("ExecMean.txt");
//     f << fixed;
//     //Report the mean and std of each one
//     std::cout << std::endl << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
//     f << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
//     cout << "OpenCV version: " << CV_VERSION << endl;
//     f << "OpenCV version: " << CV_VERSION << endl;
//     std::cout << "---------------------------" << std::endl;
//     std::cout << "Tracking" << std::setprecision(5) << std::endl << std::endl;
//     f << "---------------------------" << std::endl;
//     f << "Tracking" << std::setprecision(5) << std::endl << std::endl;
//     double average, deviation;
//     if(!vdRectStereo_ms.empty())
//     {
//         average = calcAverage(vdRectStereo_ms);
//         deviation = calcDeviation(vdRectStereo_ms, average);
//         std::cout << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
//         f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
//     }

//     if(!vdResizeImage_ms.empty())
//     {
//         average = calcAverage(vdResizeImage_ms);
//         deviation = calcDeviation(vdResizeImage_ms, average);
//         std::cout << "Image Resize: " << average << "$\\pm$" << deviation << std::endl;
//         f << "Image Resize: " << average << "$\\pm$" << deviation << std::endl;
//     }

//     average = calcAverage(vdORBExtract_ms);
//     deviation = calcDeviation(vdORBExtract_ms, average);
//     std::cout << "ORB Extraction: " << average << "$\\pm$" << deviation << std::endl;
//     f << "ORB Extraction: " << average << "$\\pm$" << deviation << std::endl;

//     if(!vdStereoMatch_ms.empty())
//     {
//         average = calcAverage(vdStereoMatch_ms);
//         deviation = calcDeviation(vdStereoMatch_ms, average);
//         std::cout << "Stereo Matching: " << average << "$\\pm$" << deviation << std::endl;
//         f << "Stereo Matching: " << average << "$\\pm$" << deviation << std::endl;
//     }

//     if(!vdIMUInteg_ms.empty())
//     {
//         average = calcAverage(vdIMUInteg_ms);
//         deviation = calcDeviation(vdIMUInteg_ms, average);
//         std::cout << "IMU Preintegration: " << average << "$\\pm$" << deviation << std::endl;
//         f << "IMU Preintegration: " << average << "$\\pm$" << deviation << std::endl;
//     }

//     average = calcAverage(vdPosePred_ms);
//     deviation = calcDeviation(vdPosePred_ms, average);
//     std::cout << "Pose Prediction: " << average << "$\\pm$" << deviation << std::endl;
//     f << "Pose Prediction: " << average << "$\\pm$" << deviation << std::endl;

//     average = calcAverage(vdLMTrack_ms);
//     deviation = calcDeviation(vdLMTrack_ms, average);
//     std::cout << "LM Track: " << average << "$\\pm$" << deviation << std::endl;
//     f << "LM Track: " << average << "$\\pm$" << deviation << std::endl;

//     average = calcAverage(vdNewKF_ms);
//     deviation = calcDeviation(vdNewKF_ms, average);
//     std::cout << "New KF decision: " << average << "$\\pm$" << deviation << std::endl;
//     f << "New KF decision: " << average << "$\\pm$" << deviation << std::endl;

//     average = calcAverage(vdTrackTotal_ms);
//     deviation = calcDeviation(vdTrackTotal_ms, average);
//     std::cout << "Total Tracking: " << average << "$\\pm$" << deviation << std::endl;
//     f << "Total Tracking: " << average << "$\\pm$" << deviation << std::endl;

//     // Local Mapping time stats
//     std::cout << std::endl << std::endl << std::endl;
//     std::cout << "Local Mapping" << std::endl << std::endl;
//     f << std::endl << "Local Mapping" << std::endl << std::endl;

//     average = calcAverage(mpLocalMapper->vdKFInsert_ms);
//     deviation = calcDeviation(mpLocalMapper->vdKFInsert_ms, average);
//     std::cout << "KF Insertion: " << average << "$\\pm$" << deviation << std::endl;
//     f << "KF Insertion: " << average << "$\\pm$" << deviation << std::endl;

//     average = calcAverage(mpLocalMapper->vdMPCulling_ms);
//     deviation = calcDeviation(mpLocalMapper->vdMPCulling_ms, average);
//     std::cout << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;
//     f << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;

//     average = calcAverage(mpLocalMapper->vdMPCreation_ms);
//     deviation = calcDeviation(mpLocalMapper->vdMPCreation_ms, average);
//     std::cout << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;
//     f << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;

//     average = calcAverage(mpLocalMapper->vdLBA_ms);
//     deviation = calcDeviation(mpLocalMapper->vdLBA_ms, average);
//     std::cout << "LBA: " << average << "$\\pm$" << deviation << std::endl;
//     f << "LBA: " << average << "$\\pm$" << deviation << std::endl;

//     average = calcAverage(mpLocalMapper->vdKFCulling_ms);
//     deviation = calcDeviation(mpLocalMapper->vdKFCulling_ms, average);
//     std::cout << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;
//     f << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;

//     average = calcAverage(mpLocalMapper->vdLMTotal_ms);
//     deviation = calcDeviation(mpLocalMapper->vdLMTotal_ms, average);
//     std::cout << "Total Local Mapping: " << average << "$\\pm$" << deviation << std::endl;
//     f << "Total Local Mapping: " << average << "$\\pm$" << deviation << std::endl;

//     // Local Mapping LBA complexity
//     std::cout << "---------------------------" << std::endl;
//     std::cout << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;
//     f << "---------------------------" << std::endl;
//     f << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;

//     average = calcAverage(mpLocalMapper->vnLBA_edges);
//     deviation = calcDeviation(mpLocalMapper->vnLBA_edges, average);
//     std::cout << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;
//     f << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;

//     average = calcAverage(mpLocalMapper->vnLBA_KFopt);
//     deviation = calcDeviation(mpLocalMapper->vnLBA_KFopt, average);
//     std::cout << "LBA KF optimized: " << average << "$\\pm$" << deviation << std::endl;
//     f << "LBA KF optimized: " << average << "$\\pm$" << deviation << std::endl;

//     average = calcAverage(mpLocalMapper->vnLBA_KFfixed);
//     deviation = calcDeviation(mpLocalMapper->vnLBA_KFfixed, average);
//     std::cout << "LBA KF fixed: " << average << "$\\pm$" << deviation << std::endl;
//     f << "LBA KF fixed: " << average << "$\\pm$" << deviation << std::endl;

//     average = calcAverage(mpLocalMapper->vnLBA_MPs);
//     deviation = calcDeviation(mpLocalMapper->vnLBA_MPs, average);
//     std::cout << "LBA MP: " << average << "$\\pm$" << deviation << std::endl << std::endl;
//     f << "LBA MP: " << average << "$\\pm$" << deviation << std::endl << std::endl;

//     std::cout << "LBA executions: " << mpLocalMapper->nLBA_exec << std::endl;
//     std::cout << "LBA aborts: " << mpLocalMapper->nLBA_abort << std::endl;
//     f << "LBA executions: " << mpLocalMapper->nLBA_exec << std::endl;
//     f << "LBA aborts: " << mpLocalMapper->nLBA_abort << std::endl;

//     // Map complexity
//     std::cout << "---------------------------" << std::endl;
//     std::cout << std::endl << "Map complexity" << std::endl;
//     std::cout << "KFs in map: " << mpAtlas->GetAllKeyFrames().size() << std::endl;
//     std::cout << "MPs in map: " << mpAtlas->GetAllMapPoints().size() << std::endl;
//     f << "---------------------------" << std::endl;
//     f << std::endl << "Map complexity" << std::endl;
//     vector<Map*> vpMaps = mpAtlas->GetAllMaps();
//     Map* pBestMap = vpMaps[0];
//     for(int i=1; i<vpMaps.size(); ++i)
//     {
//         if(pBestMap->GetAllKeyFrames().size() < vpMaps[i]->GetAllKeyFrames().size())
//         {
//             pBestMap = vpMaps[i];
//         }
//     }

//     f << "KFs in map: " << pBestMap->GetAllKeyFrames().size() << std::endl;
//     f << "MPs in map: " << pBestMap->GetAllMapPoints().size() << std::endl;

//     f << "---------------------------" << std::endl;
//     f << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl;
//     std::cout << "---------------------------" << std::endl;
//     std::cout << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl;
//     average = calcAverage(mpLoopClosing->vdDataQuery_ms);
//     deviation = calcDeviation(mpLoopClosing->vdDataQuery_ms, average);
//     f << "Database Query: " << average << "$\\pm$" << deviation << std::endl;
//     std::cout << "Database Query: " << average << "$\\pm$" << deviation << std::endl;
//     average = calcAverage(mpLoopClosing->vdEstSim3_ms);
//     deviation = calcDeviation(mpLoopClosing->vdEstSim3_ms, average);
//     f << "SE3 estimation: " << average << "$\\pm$" << deviation << std::endl;
//     std::cout << "SE3 estimation: " << average << "$\\pm$" << deviation << std::endl;
//     average = calcAverage(mpLoopClosing->vdPRTotal_ms);
//     deviation = calcDeviation(mpLoopClosing->vdPRTotal_ms, average);
//     f << "Total Place Recognition: " << average << "$\\pm$" << deviation << std::endl << std::endl;
//     std::cout << "Total Place Recognition: " << average << "$\\pm$" << deviation << std::endl << std::endl;

//     f << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl;
//     std::cout << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl;
//     average = calcAverage(mpLoopClosing->vdLoopFusion_ms);
//     deviation = calcDeviation(mpLoopClosing->vdLoopFusion_ms, average);
//     f << "Loop Fusion: " << average << "$\\pm$" << deviation << std::endl;
//     std::cout << "Loop Fusion: " << average << "$\\pm$" << deviation << std::endl;
//     average = calcAverage(mpLoopClosing->vdLoopOptEss_ms);
//     deviation = calcDeviation(mpLoopClosing->vdLoopOptEss_ms, average);
//     f << "Essential Graph: " << average << "$\\pm$" << deviation << std::endl;
//     std::cout << "Essential Graph: " << average << "$\\pm$" << deviation << std::endl;
//     average = calcAverage(mpLoopClosing->vdLoopTotal_ms);
//     deviation = calcDeviation(mpLoopClosing->vdLoopTotal_ms, average);
//     f << "Total Loop Closing: " << average << "$\\pm$" << deviation << std::endl << std::endl;
//     std::cout << "Total Loop Closing: " << average << "$\\pm$" << deviation << std::endl << std::endl;

//     f << "Numb exec: " << mpLoopClosing->nLoop << std::endl;
//     std::cout << "Num exec: " << mpLoopClosing->nLoop << std::endl;
//     average = calcAverage(mpLoopClosing->vnLoopKFs);
//     deviation = calcDeviation(mpLoopClosing->vnLoopKFs, average);
//     f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
//     std::cout << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;

//     f << std::endl << "Map Merging (mean$\\pm$std)" << std::endl;
//     std::cout << std::endl << "Map Merging (mean$\\pm$std)" << std::endl;
//     average = calcAverage(mpLoopClosing->vdMergeMaps_ms);
//     deviation = calcDeviation(mpLoopClosing->vdMergeMaps_ms, average);
//     f << "Merge Maps: " << average << "$\\pm$" << deviation << std::endl;
//     std::cout << "Merge Maps: " << average << "$\\pm$" << deviation << std::endl;
//     average = calcAverage(mpLoopClosing->vdWeldingBA_ms);
//     deviation = calcDeviation(mpLoopClosing->vdWeldingBA_ms, average);
//     f << "Welding BA: " << average << "$\\pm$" << deviation << std::endl;
//     std::cout << "Welding BA: " << average << "$\\pm$" << deviation << std::endl;
//     average = calcAverage(mpLoopClosing->vdMergeOptEss_ms);
//     deviation = calcDeviation(mpLoopClosing->vdMergeOptEss_ms, average);
//     f << "Optimization Ess.: " << average << "$\\pm$" << deviation << std::endl;
//     std::cout << "Optimization Ess.: " << average << "$\\pm$" << deviation << std::endl;
//     average = calcAverage(mpLoopClosing->vdMergeTotal_ms);
//     deviation = calcDeviation(mpLoopClosing->vdMergeTotal_ms, average);
//     f << "Total Map Merging: " << average << "$\\pm$" << deviation << std::endl << std::endl;
//     std::cout << "Total Map Merging: " << average << "$\\pm$" << deviation << std::endl << std::endl;

//     f << "Numb exec: " << mpLoopClosing->nMerges << std::endl;
//     std::cout << "Num exec: " << mpLoopClosing->nMerges << std::endl;
//     average = calcAverage(mpLoopClosing->vnMergeKFs);
//     deviation = calcDeviation(mpLoopClosing->vnMergeKFs, average);
//     f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
//     std::cout << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
//     average = calcAverage(mpLoopClosing->vnMergeMPs);
//     deviation = calcDeviation(mpLoopClosing->vnMergeMPs, average);
//     f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
//     std::cout << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;

//     f << std::endl << "Full GBA (mean$\\pm$std)" << std::endl;
//     std::cout << std::endl << "Full GBA (mean$\\pm$std)" << std::endl;
//     average = calcAverage(mpLoopClosing->vdGBA_ms);
//     deviation = calcDeviation(mpLoopClosing->vdGBA_ms, average);
//     f << "GBA: " << average << "$\\pm$" << deviation << std::endl;
//     std::cout << "GBA: " << average << "$\\pm$" << deviation << std::endl;
//     average = calcAverage(mpLoopClosing->vdUpdateMap_ms);
//     deviation = calcDeviation(mpLoopClosing->vdUpdateMap_ms, average);
//     f << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
//     std::cout << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
//     average = calcAverage(mpLoopClosing->vdFGBATotal_ms);
//     deviation = calcDeviation(mpLoopClosing->vdFGBATotal_ms, average);
//     f << "Total Full GBA: " << average << "$\\pm$" << deviation << std::endl << std::endl;
//     std::cout << "Total Full GBA: " << average << "$\\pm$" << deviation << std::endl << std::endl;

//     f << "Numb exec: " << mpLoopClosing->nFGBA_exec << std::endl;
//     std::cout << "Num exec: " << mpLoopClosing->nFGBA_exec << std::endl;
//     f << "Numb abort: " << mpLoopClosing->nFGBA_abort << std::endl;
//     std::cout << "Num abort: " << mpLoopClosing->nFGBA_abort << std::endl;
//     average = calcAverage(mpLoopClosing->vnGBAKFs);
//     deviation = calcDeviation(mpLoopClosing->vnGBAKFs, average);
//     f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
//     std::cout << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
//     average = calcAverage(mpLoopClosing->vnGBAMPs);
//     deviation = calcDeviation(mpLoopClosing->vnGBAMPs, average);
//     f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
//     std::cout << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;

//     f.close();

// }

// #endif

Tracking::~Tracking()
{
    //f_track_stats.close();
}

/**
 * @brief 
 * 1. 设置几何相机(之后会用到封装在这里边的函数，例如极线搜索)
 * 2. 畸变参数(构造Frame时使用)
 * 2. mImageScale (没啥用)
 * 3. mK mK_ 内参矩阵 (构造Frame时使用)
 * 4. mDepthMapFactor (转depth数据类型时用)
 * 5. 一些帧率方面的设置
 * 6. ORB 的一些阈值
 */
void Tracking::newParameterLoader(Settings *settings) {
    mpCamera = settings->camera1();
    // mpCamera = mpAtlas->AddCamera(mpCamera); // Atlas里的camera也是参数加载的，都是同一个

    // Frame里的相机参数是通过这tracking传进去的，Frame里只给keypoint去畸变了
    if(settings->needToUndistort()){
        mDistCoef = settings->camera1DistortionCoef();
    }
    else{
        mDistCoef = cv::Mat::zeros(4,1,CV_32F);
    }
    
    //
    //TODO: missing image scaling and rectification
    mImageScale = 1.0f; // 摆设

    // Frame 的参数是通过这个传进去的
    // OpenCv矩阵
    mK = cv::Mat::eye(3,3,CV_32F);
    mK.at<float>(0,0) = mpCamera->getParameter(0);
    mK.at<float>(1,1) = mpCamera->getParameter(1);
    mK.at<float>(0,2) = mpCamera->getParameter(2);
    mK.at<float>(1,2) = mpCamera->getParameter(3);

    // Eigen矩阵
    mK_.setIdentity();
    mK_(0,0) = mpCamera->getParameter(0);
    mK_(1,1) = mpCamera->getParameter(1);
    mK_(0,2) = mpCamera->getParameter(2);
    mK_(1,2) = mpCamera->getParameter(3);

    // if((mSensor==System::STEREO || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD) &&
    //     settings->cameraType() == Settings::KannalaBrandt){
    //     mpCamera2 = settings->camera2();
    //     mpCamera2 = mpAtlas->AddCamera(mpCamera2);

    //     mTlr = settings->Tlr();

    //     mpFrameDrawer->both = true;
    // }

    // if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD ){
    //     // 应该是基线
    //     mbf = settings->bf();
    //     mThDepth = settings->b() * settings->thDepth();
    // }

    // 如果是RGBD相机
    // if(mSensor==System::RGBD || mSensor==System::IMU_RGBD){
        mDepthMapFactor = settings->depthMapFactor();
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    // }

    // mMinFrames = 0;
    // mMaxFrames = settings->fps();
    mbRGB = settings->rgb();

    //ORB parameters
    int nFeatures = settings->nFeatures(); // Number of features per image 
    int nLevels = settings->nLevels(); // Number of levels in the scale pyramid
    int fIniThFAST = settings->initThFAST(); // Initial threshold for the FAST corner detector
    int fMinThFAST = settings->minThFAST(); // Minimum threshold for the FAST corner detector
    float fScaleFactor = settings->scaleFactor(); // Scale factor between levels in the scale pyramid

    // 创建ORB特征提取器
    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    // if(mSensor==System::STEREO || mSensor==System::IMU_STEREO)
    //     mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    // if(mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR)
    //     mpIniORBextractor = new ORBextractor(5*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    // // IMU parameters
    // Sophus::SE3f Tbc = settings->Tbc();
    // mInsertKFsLost = settings->insertKFsWhenLost();
    // mImuFreq = settings->imuFrequency();
    // mImuPer = 0.001; //1.0 / (double) mImuFreq;     //TODO: ESTO ESTA BIEN?
    // float Ng = settings->noiseGyro();
    // float Na = settings->noiseAcc();
    // float Ngw = settings->gyroWalk();
    // float Naw = settings->accWalk();

    // const float sf = sqrt(mImuFreq);
    // mpImuCalib = new IMU::Calib(Tbc,Ng*sf,Na*sf,Ngw/sf,Naw/sf);

    // mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
}

// bool Tracking::ParseCamParamFile(cv::FileStorage &fSettings)
// {
//     mDistCoef = cv::Mat::zeros(4,1,CV_32F);
//     cout << endl << "Camera Parameters: " << endl;
//     bool b_miss_params = false;

//     string sCameraName = fSettings["Camera.type"];
//     if(sCameraName == "PinHole")
//     {
//         float fx, fy, cx, cy;
//         mImageScale = 1.f;

//         // Camera calibration parameters
//         cv::FileNode node = fSettings["Camera.fx"];
//         if(!node.empty() && node.isReal())
//         {
//             fx = node.real();
//         }
//         else
//         {
//             std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
//             b_miss_params = true;
//         }

//         node = fSettings["Camera.fy"];
//         if(!node.empty() && node.isReal())
//         {
//             fy = node.real();
//         }
//         else
//         {
//             std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
//             b_miss_params = true;
//         }

//         node = fSettings["Camera.cx"];
//         if(!node.empty() && node.isReal())
//         {
//             cx = node.real();
//         }
//         else
//         {
//             std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
//             b_miss_params = true;
//         }

//         node = fSettings["Camera.cy"];
//         if(!node.empty() && node.isReal())
//         {
//             cy = node.real();
//         }
//         else
//         {
//             std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
//             b_miss_params = true;
//         }

//         // Distortion parameters
//         node = fSettings["Camera.k1"];
//         if(!node.empty() && node.isReal())
//         {
//             mDistCoef.at<float>(0) = node.real();
//         }
//         else
//         {
//             std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
//             b_miss_params = true;
//         }

//         node = fSettings["Camera.k2"];
//         if(!node.empty() && node.isReal())
//         {
//             mDistCoef.at<float>(1) = node.real();
//         }
//         else
//         {
//             std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
//             b_miss_params = true;
//         }

//         node = fSettings["Camera.p1"];
//         if(!node.empty() && node.isReal())
//         {
//             mDistCoef.at<float>(2) = node.real();
//         }
//         else
//         {
//             std::cerr << "*Camera.p1 parameter doesn't exist or is not a real number*" << std::endl;
//             b_miss_params = true;
//         }

//         node = fSettings["Camera.p2"];
//         if(!node.empty() && node.isReal())
//         {
//             mDistCoef.at<float>(3) = node.real();
//         }
//         else
//         {
//             std::cerr << "*Camera.p2 parameter doesn't exist or is not a real number*" << std::endl;
//             b_miss_params = true;
//         }

//         node = fSettings["Camera.k3"];
//         if(!node.empty() && node.isReal())
//         {
//             mDistCoef.resize(5);
//             mDistCoef.at<float>(4) = node.real();
//         }

//         node = fSettings["Camera.imageScale"];
//         if(!node.empty() && node.isReal())
//         {
//             mImageScale = node.real();
//         }

//         if(b_miss_params)
//         {
//             return false;
//         }

//         if(mImageScale != 1.f)
//         {
//             // K matrix parameters must be scaled.
//             fx = fx * mImageScale;
//             fy = fy * mImageScale;
//             cx = cx * mImageScale;
//             cy = cy * mImageScale;
//         }

//         vector<float> vCamCalib{fx,fy,cx,cy};

//         mpCamera = new Pinhole(vCamCalib);

//         mpCamera = mpAtlas->AddCamera(mpCamera);

//         std::cout << "- Camera: Pinhole" << std::endl;
//         std::cout << "- Image scale: " << mImageScale << std::endl;
//         std::cout << "- fx: " << fx << std::endl;
//         std::cout << "- fy: " << fy << std::endl;
//         std::cout << "- cx: " << cx << std::endl;
//         std::cout << "- cy: " << cy << std::endl;
//         std::cout << "- k1: " << mDistCoef.at<float>(0) << std::endl;
//         std::cout << "- k2: " << mDistCoef.at<float>(1) << std::endl;


//         std::cout << "- p1: " << mDistCoef.at<float>(2) << std::endl;
//         std::cout << "- p2: " << mDistCoef.at<float>(3) << std::endl;

//         if(mDistCoef.rows==5)
//             std::cout << "- k3: " << mDistCoef.at<float>(4) << std::endl;

//         mK = cv::Mat::eye(3,3,CV_32F);
//         mK.at<float>(0,0) = fx;
//         mK.at<float>(1,1) = fy;
//         mK.at<float>(0,2) = cx;
//         mK.at<float>(1,2) = cy;

//         mK_.setIdentity();
//         mK_(0,0) = fx;
//         mK_(1,1) = fy;
//         mK_(0,2) = cx;
//         mK_(1,2) = cy;
//     }
//     else if(sCameraName == "KannalaBrandt8")
//     {
//         float fx, fy, cx, cy;
//         float k1, k2, k3, k4;
//         mImageScale = 1.f;

//         // Camera calibration parameters
//         cv::FileNode node = fSettings["Camera.fx"];
//         if(!node.empty() && node.isReal())
//         {
//             fx = node.real();
//         }
//         else
//         {
//             std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
//             b_miss_params = true;
//         }
//         node = fSettings["Camera.fy"];
//         if(!node.empty() && node.isReal())
//         {
//             fy = node.real();
//         }
//         else
//         {
//             std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
//             b_miss_params = true;
//         }

//         node = fSettings["Camera.cx"];
//         if(!node.empty() && node.isReal())
//         {
//             cx = node.real();
//         }
//         else
//         {
//             std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
//             b_miss_params = true;
//         }

//         node = fSettings["Camera.cy"];
//         if(!node.empty() && node.isReal())
//         {
//             cy = node.real();
//         }
//         else
//         {
//             std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
//             b_miss_params = true;
//         }

//         // Distortion parameters
//         node = fSettings["Camera.k1"];
//         if(!node.empty() && node.isReal())
//         {
//             k1 = node.real();
//         }
//         else
//         {
//             std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
//             b_miss_params = true;
//         }
//         node = fSettings["Camera.k2"];
//         if(!node.empty() && node.isReal())
//         {
//             k2 = node.real();
//         }
//         else
//         {
//             std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
//             b_miss_params = true;
//         }

//         node = fSettings["Camera.k3"];
//         if(!node.empty() && node.isReal())
//         {
//             k3 = node.real();
//         }
//         else
//         {
//             std::cerr << "*Camera.k3 parameter doesn't exist or is not a real number*" << std::endl;
//             b_miss_params = true;
//         }

//         node = fSettings["Camera.k4"];
//         if(!node.empty() && node.isReal())
//         {
//             k4 = node.real();
//         }
//         else
//         {
//             std::cerr << "*Camera.k4 parameter doesn't exist or is not a real number*" << std::endl;
//             b_miss_params = true;
//         }

//         node = fSettings["Camera.imageScale"];
//         if(!node.empty() && node.isReal())
//         {
//             mImageScale = node.real();
//         }

//         if(!b_miss_params)
//         {
//             if(mImageScale != 1.f)
//             {
//                 // K matrix parameters must be scaled.
//                 fx = fx * mImageScale;
//                 fy = fy * mImageScale;
//                 cx = cx * mImageScale;
//                 cy = cy * mImageScale;
//             }

//             vector<float> vCamCalib{fx,fy,cx,cy,k1,k2,k3,k4};
//             mpCamera = new KannalaBrandt8(vCamCalib);
//             mpCamera = mpAtlas->AddCamera(mpCamera);
//             std::cout << "- Camera: Fisheye" << std::endl;
//             std::cout << "- Image scale: " << mImageScale << std::endl;
//             std::cout << "- fx: " << fx << std::endl;
//             std::cout << "- fy: " << fy << std::endl;
//             std::cout << "- cx: " << cx << std::endl;
//             std::cout << "- cy: " << cy << std::endl;
//             std::cout << "- k1: " << k1 << std::endl;
//             std::cout << "- k2: " << k2 << std::endl;
//             std::cout << "- k3: " << k3 << std::endl;
//             std::cout << "- k4: " << k4 << std::endl;

//             mK = cv::Mat::eye(3,3,CV_32F);
//             mK.at<float>(0,0) = fx;
//             mK.at<float>(1,1) = fy;
//             mK.at<float>(0,2) = cx;
//             mK.at<float>(1,2) = cy;

//             mK_.setIdentity();
//             mK_(0,0) = fx;
//             mK_(1,1) = fy;
//             mK_(0,2) = cx;
//             mK_(1,2) = cy;
//         }

//         if(mSensor==System::STEREO || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD){
//             // Right camera
//             // Camera calibration parameters
//             cv::FileNode node = fSettings["Camera2.fx"];
//             if(!node.empty() && node.isReal())
//             {
//                 fx = node.real();
//             }
//             else
//             {
//                 std::cerr << "*Camera2.fx parameter doesn't exist or is not a real number*" << std::endl;
//                 b_miss_params = true;
//             }
//             node = fSettings["Camera2.fy"];
//             if(!node.empty() && node.isReal())
//             {
//                 fy = node.real();
//             }
//             else
//             {
//                 std::cerr << "*Camera2.fy parameter doesn't exist or is not a real number*" << std::endl;
//                 b_miss_params = true;
//             }

//             node = fSettings["Camera2.cx"];
//             if(!node.empty() && node.isReal())
//             {
//                 cx = node.real();
//             }
//             else
//             {
//                 std::cerr << "*Camera2.cx parameter doesn't exist or is not a real number*" << std::endl;
//                 b_miss_params = true;
//             }

//             node = fSettings["Camera2.cy"];
//             if(!node.empty() && node.isReal())
//             {
//                 cy = node.real();
//             }
//             else
//             {
//                 std::cerr << "*Camera2.cy parameter doesn't exist or is not a real number*" << std::endl;
//                 b_miss_params = true;
//             }

//             // Distortion parameters
//             node = fSettings["Camera2.k1"];
//             if(!node.empty() && node.isReal())
//             {
//                 k1 = node.real();
//             }
//             else
//             {
//                 std::cerr << "*Camera2.k1 parameter doesn't exist or is not a real number*" << std::endl;
//                 b_miss_params = true;
//             }
//             node = fSettings["Camera2.k2"];
//             if(!node.empty() && node.isReal())
//             {
//                 k2 = node.real();
//             }
//             else
//             {
//                 std::cerr << "*Camera2.k2 parameter doesn't exist or is not a real number*" << std::endl;
//                 b_miss_params = true;
//             }

//             node = fSettings["Camera2.k3"];
//             if(!node.empty() && node.isReal())
//             {
//                 k3 = node.real();
//             }
//             else
//             {
//                 std::cerr << "*Camera2.k3 parameter doesn't exist or is not a real number*" << std::endl;
//                 b_miss_params = true;
//             }

//             node = fSettings["Camera2.k4"];
//             if(!node.empty() && node.isReal())
//             {
//                 k4 = node.real();
//             }
//             else
//             {
//                 std::cerr << "*Camera2.k4 parameter doesn't exist or is not a real number*" << std::endl;
//                 b_miss_params = true;
//             }


//             int leftLappingBegin = -1;
//             int leftLappingEnd = -1;

//             int rightLappingBegin = -1;
//             int rightLappingEnd = -1;

//             node = fSettings["Camera.lappingBegin"];
//             if(!node.empty() && node.isInt())
//             {
//                 leftLappingBegin = node.operator int();
//             }
//             else
//             {
//                 std::cout << "WARNING: Camera.lappingBegin not correctly defined" << std::endl;
//             }
//             node = fSettings["Camera.lappingEnd"];
//             if(!node.empty() && node.isInt())
//             {
//                 leftLappingEnd = node.operator int();
//             }
//             else
//             {
//                 std::cout << "WARNING: Camera.lappingEnd not correctly defined" << std::endl;
//             }
//             node = fSettings["Camera2.lappingBegin"];
//             if(!node.empty() && node.isInt())
//             {
//                 rightLappingBegin = node.operator int();
//             }
//             else
//             {
//                 std::cout << "WARNING: Camera2.lappingBegin not correctly defined" << std::endl;
//             }
//             node = fSettings["Camera2.lappingEnd"];
//             if(!node.empty() && node.isInt())
//             {
//                 rightLappingEnd = node.operator int();
//             }
//             else
//             {
//                 std::cout << "WARNING: Camera2.lappingEnd not correctly defined" << std::endl;
//             }

//             node = fSettings["Tlr"];
//             cv::Mat cvTlr;
//             if(!node.empty())
//             {
//                 cvTlr = node.mat();
//                 if(cvTlr.rows != 3 || cvTlr.cols != 4)
//                 {
//                     std::cerr << "*Tlr matrix have to be a 3x4 transformation matrix*" << std::endl;
//                     b_miss_params = true;
//                 }
//             }
//             else
//             {
//                 std::cerr << "*Tlr matrix doesn't exist*" << std::endl;
//                 b_miss_params = true;
//             }

//             if(!b_miss_params)
//             {
//                 if(mImageScale != 1.f)
//                 {
//                     // K matrix parameters must be scaled.
//                     fx = fx * mImageScale;
//                     fy = fy * mImageScale;
//                     cx = cx * mImageScale;
//                     cy = cy * mImageScale;

//                     leftLappingBegin = leftLappingBegin * mImageScale;
//                     leftLappingEnd = leftLappingEnd * mImageScale;
//                     rightLappingBegin = rightLappingBegin * mImageScale;
//                     rightLappingEnd = rightLappingEnd * mImageScale;
//                 }

//                 static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[0] = leftLappingBegin;
//                 static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[1] = leftLappingEnd;

//                 mpFrameDrawer->both = true;

//                 vector<float> vCamCalib2{fx,fy,cx,cy,k1,k2,k3,k4};
//                 mpCamera2 = new KannalaBrandt8(vCamCalib2);
//                 mpCamera2 = mpAtlas->AddCamera(mpCamera2);

//                 mTlr = Converter::toSophus(cvTlr);

//                 static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[0] = rightLappingBegin;
//                 static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[1] = rightLappingEnd;

//                 std::cout << "- Camera1 Lapping: " << leftLappingBegin << ", " << leftLappingEnd << std::endl;

//                 std::cout << std::endl << "Camera2 Parameters:" << std::endl;
//                 std::cout << "- Camera: Fisheye" << std::endl;
//                 std::cout << "- Image scale: " << mImageScale << std::endl;
//                 std::cout << "- fx: " << fx << std::endl;
//                 std::cout << "- fy: " << fy << std::endl;
//                 std::cout << "- cx: " << cx << std::endl;
//                 std::cout << "- cy: " << cy << std::endl;
//                 std::cout << "- k1: " << k1 << std::endl;
//                 std::cout << "- k2: " << k2 << std::endl;
//                 std::cout << "- k3: " << k3 << std::endl;
//                 std::cout << "- k4: " << k4 << std::endl;

//                 std::cout << "- mTlr: \n" << cvTlr << std::endl;

//                 std::cout << "- Camera2 Lapping: " << rightLappingBegin << ", " << rightLappingEnd << std::endl;
//             }
//         }

//         if(b_miss_params)
//         {
//             return false;
//         }

//     }
//     else
//     {
//         std::cerr << "*Not Supported Camera Sensor*" << std::endl;
//         std::cerr << "Check an example configuration file with the desired sensor" << std::endl;
//     }

//     if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD )
//     {
//         cv::FileNode node = fSettings["Camera.bf"];
//         if(!node.empty() && node.isReal())
//         {
//             mbf = node.real();
//             if(mImageScale != 1.f)
//             {
//                 mbf *= mImageScale;
//             }
//         }
//         else
//         {
//             std::cerr << "*Camera.bf parameter doesn't exist or is not a real number*" << std::endl;
//             b_miss_params = true;
//         }

//     }

//     float fps = fSettings["Camera.fps"];
//     if(fps==0)
//         fps=30;

//     // Max/Min Frames to insert keyframes and to check relocalisation
//     mMinFrames = 0;
//     mMaxFrames = fps;

//     cout << "- fps: " << fps << endl;


//     int nRGB = fSettings["Camera.RGB"];
//     mbRGB = nRGB;

//     if(mbRGB)
//         cout << "- color order: RGB (ignored if grayscale)" << endl;
//     else
//         cout << "- color order: BGR (ignored if grayscale)" << endl;

//     if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD)
//     {
//         float fx = mpCamera->getParameter(0);
//         cv::FileNode node = fSettings["ThDepth"];
//         if(!node.empty()  && node.isReal())
//         {
//             mThDepth = node.real();
//             mThDepth = mbf*mThDepth/fx;
//             cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
//         }
//         else
//         {
//             std::cerr << "*ThDepth parameter doesn't exist or is not a real number*" << std::endl;
//             b_miss_params = true;
//         }


//     }

//     if(mSensor==System::RGBD || mSensor==System::IMU_RGBD)
//     {
//         cv::FileNode node = fSettings["DepthMapFactor"];
//         if(!node.empty() && node.isReal())
//         {
//             mDepthMapFactor = node.real();
//             if(fabs(mDepthMapFactor)<1e-5)
//                 mDepthMapFactor=1;
//             else
//                 mDepthMapFactor = 1.0f/mDepthMapFactor;
//         }
//         else
//         {
//             std::cerr << "*DepthMapFactor parameter doesn't exist or is not a real number*" << std::endl;
//             b_miss_params = true;
//         }

//     }

//     if(b_miss_params)
//     {
//         return false;
//     }

//     return true;
// }

// bool Tracking::ParseORBParamFile(cv::FileStorage &fSettings)
// {
//     bool b_miss_params = false;
//     int nFeatures, nLevels, fIniThFAST, fMinThFAST;
//     float fScaleFactor;

//     cv::FileNode node = fSettings["ORBextractor.nFeatures"];
//     if(!node.empty() && node.isInt())
//     {
//         nFeatures = node.operator int();
//     }
//     else
//     {
//         std::cerr << "*ORBextractor.nFeatures parameter doesn't exist or is not an integer*" << std::endl;
//         b_miss_params = true;
//     }

//     node = fSettings["ORBextractor.scaleFactor"];
//     if(!node.empty() && node.isReal())
//     {
//         fScaleFactor = node.real();
//     }
//     else
//     {
//         std::cerr << "*ORBextractor.scaleFactor parameter doesn't exist or is not a real number*" << std::endl;
//         b_miss_params = true;
//     }

//     node = fSettings["ORBextractor.nLevels"];
//     if(!node.empty() && node.isInt())
//     {
//         nLevels = node.operator int();
//     }
//     else
//     {
//         std::cerr << "*ORBextractor.nLevels parameter doesn't exist or is not an integer*" << std::endl;
//         b_miss_params = true;
//     }

//     node = fSettings["ORBextractor.iniThFAST"];
//     if(!node.empty() && node.isInt())
//     {
//         fIniThFAST = node.operator int();
//     }
//     else
//     {
//         std::cerr << "*ORBextractor.iniThFAST parameter doesn't exist or is not an integer*" << std::endl;
//         b_miss_params = true;
//     }

//     node = fSettings["ORBextractor.minThFAST"];
//     if(!node.empty() && node.isInt())
//     {
//         fMinThFAST = node.operator int();
//     }
//     else
//     {
//         std::cerr << "*ORBextractor.minThFAST parameter doesn't exist or is not an integer*" << std::endl;
//         b_miss_params = true;
//     }

//     if(b_miss_params)
//     {
//         return false;
//     }

//     mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

//     if(mSensor==System::STEREO || mSensor==System::IMU_STEREO)
//         mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

//     if(mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR)
//         mpIniORBextractor = new ORBextractor(5*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

//     cout << endl << "ORB Extractor Parameters: " << endl;
//     cout << "- Number of Features: " << nFeatures << endl;
//     cout << "- Scale Levels: " << nLevels << endl;
//     cout << "- Scale Factor: " << fScaleFactor << endl;
//     cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
//     cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

//     return true;
// }

// bool Tracking::ParseIMUParamFile(cv::FileStorage &fSettings)
// {
//     bool b_miss_params = false;

//     cv::Mat cvTbc;
//     cv::FileNode node = fSettings["Tbc"];
//     if(!node.empty())
//     {
//         cvTbc = node.mat();
//         if(cvTbc.rows != 4 || cvTbc.cols != 4)
//         {
//             std::cerr << "*Tbc matrix have to be a 4x4 transformation matrix*" << std::endl;
//             b_miss_params = true;
//         }
//     }
//     else
//     {
//         std::cerr << "*Tbc matrix doesn't exist*" << std::endl;
//         b_miss_params = true;
//     }
//     cout << endl;
//     cout << "Left camera to Imu Transform (Tbc): " << endl << cvTbc << endl;
//     Eigen::Matrix<float,4,4,Eigen::RowMajor> eigTbc(cvTbc.ptr<float>(0));
//     Sophus::SE3f Tbc(eigTbc);

//     node = fSettings["InsertKFsWhenLost"];
//     mInsertKFsLost = true;
//     if(!node.empty() && node.isInt())
//     {
//         mInsertKFsLost = (bool) node.operator int();
//     }

//     if(!mInsertKFsLost)
//         cout << "Do not insert keyframes when lost visual tracking " << endl;



//     float Ng, Na, Ngw, Naw;

//     node = fSettings["IMU.Frequency"];
//     if(!node.empty() && node.isInt())
//     {
//         mImuFreq = node.operator int();
//         mImuPer = 0.001; //1.0 / (double) mImuFreq;
//     }
//     else
//     {
//         std::cerr << "*IMU.Frequency parameter doesn't exist or is not an integer*" << std::endl;
//         b_miss_params = true;
//     }

//     node = fSettings["IMU.NoiseGyro"];
//     if(!node.empty() && node.isReal())
//     {
//         Ng = node.real();
//     }
//     else
//     {
//         std::cerr << "*IMU.NoiseGyro parameter doesn't exist or is not a real number*" << std::endl;
//         b_miss_params = true;
//     }

//     node = fSettings["IMU.NoiseAcc"];
//     if(!node.empty() && node.isReal())
//     {
//         Na = node.real();
//     }
//     else
//     {
//         std::cerr << "*IMU.NoiseAcc parameter doesn't exist or is not a real number*" << std::endl;
//         b_miss_params = true;
//     }

//     node = fSettings["IMU.GyroWalk"];
//     if(!node.empty() && node.isReal())
//     {
//         Ngw = node.real();
//     }
//     else
//     {
//         std::cerr << "*IMU.GyroWalk parameter doesn't exist or is not a real number*" << std::endl;
//         b_miss_params = true;
//     }

//     node = fSettings["IMU.AccWalk"];
//     if(!node.empty() && node.isReal())
//     {
//         Naw = node.real();
//     }
//     else
//     {
//         std::cerr << "*IMU.AccWalk parameter doesn't exist or is not a real number*" << std::endl;
//         b_miss_params = true;
//     }

//     node = fSettings["IMU.fastInit"];
//     mFastInit = false;
//     if(!node.empty())
//     {
//         mFastInit = static_cast<int>(fSettings["IMU.fastInit"]) != 0;
//     }

//     if(mFastInit)
//         cout << "Fast IMU initialization. Acceleration is not checked \n";

//     if(b_miss_params)
//     {
//         return false;
//     }

//     const float sf = sqrt(mImuFreq);
//     cout << endl;
//     cout << "IMU frequency: " << mImuFreq << " Hz" << endl;
//     cout << "IMU gyro noise: " << Ng << " rad/s/sqrt(Hz)" << endl;
//     cout << "IMU gyro walk: " << Ngw << " rad/s^2/sqrt(Hz)" << endl;
//     cout << "IMU accelerometer noise: " << Na << " m/s^2/sqrt(Hz)" << endl;
//     cout << "IMU accelerometer walk: " << Naw << " m/s^3/sqrt(Hz)" << endl;

//     mpImuCalib = new IMU::Calib(Tbc,Ng*sf,Na*sf,Ngw/sf,Naw/sf);

//     mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);


//     return true;
// }

// void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
// {
//     mpLocalMapper=pLocalMapper;
// }

// void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
// {
//     mpLoopClosing=pLoopClosing;
// }

// void Tracking::SetViewer(Viewer *pViewer)
// {
//     mpViewer=pViewer;
// }

// void Tracking::SetStepByStep(bool bSet)
// {
//     bStepByStep = bSet;
// }

// bool Tracking::GetStepByStep()
// {
//     return bStepByStep;
// }

/**
 * @brief 制作frame,并进行track()
 * @param imRGB RGB   图片
 * @param imD   Depth 图片(要和RGB对齐)
 * 1. 创建Frame, 此过程中会提取ORB特征，并进行分配，以便加速匹配
 * 2. 进行帧间追踪
 */
void Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD)
{
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    // 转成灰度图
    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGR2GRAY);
    }
    else if(mImGray.channels()==4) // 这是什么格式？
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGRA2GRAY);
    }

    // 深度图也要转换?
    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

    /**    1.构造Frame    **/ 
    // if (mSensor == System::RGBD)
        mCurrentFrame = Frame(mImGray,imDepth,mpORBextractorLeft,mK,mDistCoef,mbf,mThDepth,mpCamera);
    // else if(mSensor == System::IMU_RGBD)
    //     mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,&mLastFrame,*mpImuCalib);






    // mCurrentFrame.mNameFile = filename;
    // mCurrentFrame.mnDataset = mnNumDataset;

// #ifdef REGISTER_TIMES
//     vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
// #endif

    /**    2.帧间追踪    **/ 
    Track();

    // return mCurrentFrame.GetPose();
}


// void Tracking::GrabImuData(const IMU::Point &imuMeasurement)
// {
//     unique_lock<mutex> lock(mMutexImuQueue);
//     mlQueueImuData.push_back(imuMeasurement);
// }

// void Tracking::PreintegrateIMU()
// {

//     if(!mCurrentFrame.mpPrevFrame)
//     {
//         Verbose::PrintMess("non prev frame ", Verbose::VERBOSITY_NORMAL);
//         mCurrentFrame.setIntegrated();
//         return;
//     }

//     mvImuFromLastFrame.clear();
//     mvImuFromLastFrame.reserve(mlQueueImuData.size());
//     if(mlQueueImuData.size() == 0)
//     {
//         Verbose::PrintMess("Not IMU data in mlQueueImuData!!", Verbose::VERBOSITY_NORMAL);
//         mCurrentFrame.setIntegrated();
//         return;
//     }

//     while(true)
//     {
//         bool bSleep = false;
//         {
//             unique_lock<mutex> lock(mMutexImuQueue);
//             if(!mlQueueImuData.empty())
//             {
//                 IMU::Point* m = &mlQueueImuData.front();
//                 cout.precision(17);
//                 if(m->t<mCurrentFrame.mpPrevFrame->mTimeStamp-mImuPer)
//                 {
//                     mlQueueImuData.pop_front();
//                 }
//                 else if(m->t<mCurrentFrame.mTimeStamp-mImuPer)
//                 {
//                     mvImuFromLastFrame.push_back(*m);
//                     mlQueueImuData.pop_front();
//                 }
//                 else
//                 {
//                     mvImuFromLastFrame.push_back(*m);
//                     break;
//                 }
//             }
//             else
//             {
//                 break;
//                 bSleep = true;
//             }
//         }
//         if(bSleep)
//             usleep(500);
//     }

//     const int n = mvImuFromLastFrame.size()-1;
//     if(n==0){
//         cout << "Empty IMU measurements vector!!!\n";
//         return;
//     }

//     IMU::Preintegrated* pImuPreintegratedFromLastFrame = new IMU::Preintegrated(mLastFrame.mImuBias,mCurrentFrame.mImuCalib);

//     for(int i=0; i<n; i++)
//     {
//         float tstep;
//         Eigen::Vector3f acc, angVel;
//         if((i==0) && (i<(n-1)))
//         {
//             float tab = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t;
//             float tini = mvImuFromLastFrame[i].t-mCurrentFrame.mpPrevFrame->mTimeStamp;
//             acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a-
//                     (mvImuFromLastFrame[i+1].a-mvImuFromLastFrame[i].a)*(tini/tab))*0.5f;
//             angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w-
//                     (mvImuFromLastFrame[i+1].w-mvImuFromLastFrame[i].w)*(tini/tab))*0.5f;
//             tstep = mvImuFromLastFrame[i+1].t-mCurrentFrame.mpPrevFrame->mTimeStamp;
//         }
//         else if(i<(n-1))
//         {
//             acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a)*0.5f;
//             angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w)*0.5f;
//             tstep = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t;
//         }
//         else if((i>0) && (i==(n-1)))
//         {
//             float tab = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t;
//             float tend = mvImuFromLastFrame[i+1].t-mCurrentFrame.mTimeStamp;
//             acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a-
//                     (mvImuFromLastFrame[i+1].a-mvImuFromLastFrame[i].a)*(tend/tab))*0.5f;
//             angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w-
//                     (mvImuFromLastFrame[i+1].w-mvImuFromLastFrame[i].w)*(tend/tab))*0.5f;
//             tstep = mCurrentFrame.mTimeStamp-mvImuFromLastFrame[i].t;
//         }
//         else if((i==0) && (i==(n-1)))
//         {
//             acc = mvImuFromLastFrame[i].a;
//             angVel = mvImuFromLastFrame[i].w;
//             tstep = mCurrentFrame.mTimeStamp-mCurrentFrame.mpPrevFrame->mTimeStamp;
//         }

//         if (!mpImuPreintegratedFromLastKF)
//             cout << "mpImuPreintegratedFromLastKF does not exist" << endl;
//         mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(acc,angVel,tstep);
//         pImuPreintegratedFromLastFrame->IntegrateNewMeasurement(acc,angVel,tstep);
//     }

//     mCurrentFrame.mpImuPreintegratedFrame = pImuPreintegratedFromLastFrame;
//     mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
//     mCurrentFrame.mpLastKeyFrame = mpLastKeyFrame;

//     mCurrentFrame.setIntegrated();

//     //Verbose::PrintMess("Preintegration is finished!! ", Verbose::VERBOSITY_DEBUG);
// }


// bool Tracking::PredictStateIMU()
// {
//     if(!mCurrentFrame.mpPrevFrame)
//     {
//         Verbose::PrintMess("No last frame", Verbose::VERBOSITY_NORMAL);
//         return false;
//     }

//     if(mbMapUpdated && mpLastKeyFrame)
//     {
//         const Eigen::Vector3f twb1 = mpLastKeyFrame->GetImuPosition();
//         const Eigen::Matrix3f Rwb1 = mpLastKeyFrame->GetImuRotation();
//         const Eigen::Vector3f Vwb1 = mpLastKeyFrame->GetVelocity();

//         const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
//         const float t12 = mpImuPreintegratedFromLastKF->dT;

//         Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaRotation(mpLastKeyFrame->GetImuBias()));
//         Eigen::Vector3f twb2 = twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mpImuPreintegratedFromLastKF->GetDeltaPosition(mpLastKeyFrame->GetImuBias());
//         Eigen::Vector3f Vwb2 = Vwb1 + t12*Gz + Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaVelocity(mpLastKeyFrame->GetImuBias());
//         mCurrentFrame.SetImuPoseVelocity(Rwb2,twb2,Vwb2);

//         mCurrentFrame.mImuBias = mpLastKeyFrame->GetImuBias();
//         mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
//         return true;
//     }
//     else if(!mbMapUpdated)
//     {
//         const Eigen::Vector3f twb1 = mLastFrame.GetImuPosition();
//         const Eigen::Matrix3f Rwb1 = mLastFrame.GetImuRotation();
//         const Eigen::Vector3f Vwb1 = mLastFrame.GetVelocity();
//         const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
//         const float t12 = mCurrentFrame.mpImuPreintegratedFrame->dT;

//         Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaRotation(mLastFrame.mImuBias));
//         Eigen::Vector3f twb2 = twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaPosition(mLastFrame.mImuBias);
//         Eigen::Vector3f Vwb2 = Vwb1 + t12*Gz + Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaVelocity(mLastFrame.mImuBias);

//         mCurrentFrame.SetImuPoseVelocity(Rwb2,twb2,Vwb2);

//         mCurrentFrame.mImuBias = mLastFrame.mImuBias;
//         mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
//         return true;
//     }
//     else
//         cout << "not IMU prediction!!" << endl;

//     return false;
// }

// void Tracking::ResetFrameIMU()
// {
//     // TODO To implement...
// }

/**
 * @brief 
 * 1. 通过运动模型预测当前帧的位姿，之后通过图优化调整位姿 TrackWithMotionModel
 * 2. 通过局部地图的共视关系，再一次优化当前帧位姿 TrackLocalMap
 * 3. 是否需要生成新的关键帧 CreateNewKeyFrame
 */
void Tracking::Track()
{
    // // 不知道这个flag作用
    // if (bStepByStep)
    // {
    //     std::cout << "Tracking: Waiting to the next step" << std::endl;
    //     while(!mbStep && bStepByStep)
    //         usleep(500);
    //     mbStep = false;
    // }
    
    // // IMU挂了
    // if(mpLocalMapper->mbBadImu)
    // {
    //     cout << "TRACK: Reset map because local mapper set the bad imu flag " << endl;
    //     mpSystem->ResetActiveMap();
    //     return;
    // }

    // // 拿到当前地图
    // Map* pCurrentMap = mpAtlas->GetCurrentMap();
    // if(!pCurrentMap)
    // {
    //     cout << "ERROR: There is not an active map in the atlas" << endl;
    // }

    // if(mState!=NO_IMAGES_YET)
    // {
    //     if(mLastFrame.mTimeStamp>mCurrentFrame.mTimeStamp)
    //     {
    //         cerr << "ERROR: Frame with a timestamp older than previous frame detected!" << endl;
    //         unique_lock<mutex> lock(mMutexImuQueue);
    //         mlQueueImuData.clear();
    //         CreateMapInAtlas();
    //         return;
    //     }
    //     else if(mCurrentFrame.mTimeStamp>mLastFrame.mTimeStamp+1.0)
    //     {
    //         // cout << mCurrentFrame.mTimeStamp << ", " << mLastFrame.mTimeStamp << endl;
    //         // cout << "id last: " << mLastFrame.mnId << "    id curr: " << mCurrentFrame.mnId << endl;
    //         if(mpAtlas->isInertial())
    //         {

    //             if(mpAtlas->isImuInitialized())
    //             {
    //                 cout << "Timestamp jump detected. State set to LOST. Reseting IMU integration..." << endl;
    //                 if(!pCurrentMap->GetIniertialBA2())
    //                 {
    //                     mpSystem->ResetActiveMap();
    //                 }
    //                 else
    //                 {
    //                     CreateMapInAtlas();
    //                 }
    //             }
    //             else
    //             {
    //                 cout << "Timestamp jump detected, before IMU initialization. Reseting..." << endl;
    //                 mpSystem->ResetActiveMap();
    //             }
    //             return;
    //         }

    //     }
    // }


    // if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && mpLastKeyFrame)
    //     mCurrentFrame.SetNewBias(mpLastKeyFrame->GetImuBias());

    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }
    // std::cout << "mstate: " << mState <<std::endl;
    mLastProcessedState=mState;

//     // 带IMU的传感器要进行预积分
//     if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && !mbCreatedMap)
//     {
// #ifdef REGISTER_TIMES
//         std::chrono::steady_clock::time_point time_StartPreIMU = std::chrono::steady_clock::now();
// #endif
//         PreintegrateIMU();
// #ifdef REGISTER_TIMES
//         std::chrono::steady_clock::time_point time_EndPreIMU = std::chrono::steady_clock::now();

//         double timePreImu = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPreIMU - time_StartPreIMU).count();
//         vdIMUInteg_ms.push_back(timePreImu);
// #endif
//     }

    // mbCreatedMap = false;

    // // Get Map Mutex -> Map cannot be changed
    // unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);

    // mbMapUpdated = false;

    // int nCurMapChangeIndex = pCurrentMap->GetMapChangeIndex();
    // int nMapChangeIndex = pCurrentMap->GetLastMapChange();
    // if(nCurMapChangeIndex>nMapChangeIndex)
    // {
    //     pCurrentMap->SetLastMapChange(nCurMapChangeIndex);
    //     mbMapUpdated = true;
    // }


    if(mState==NOT_INITIALIZED)
    {

        StereoInitialization();
        // std::cout << "size  : " << mLastFrame.mvpMapPoints.size() << std::endl;
        // for(int i = 0; i < mLastFrame.mvpMapPoints.size(); i++){
        //     ORB_SLAM3::MapPoint* p = mLastFrame.mvpMapPoints[i];
        //     if(p){
        //         auto pos = p->mnId;
        //         std::cout << pos << std::endl;
        //     }
        // }
        //mpFrameDrawer->Update(this);

        if(mState!=OK) // If rightly initialized, mState=OK
        {
            mLastFrame = Frame(mCurrentFrame);
            return;
        }
        // if(mpAtlas->GetAllMaps().size() == 1)
        // {
        //     mnFirstFrameId = mCurrentFrame.mnId;
        // }
    }
    else
    {
        std::cout << "tracking ..." << std::endl;
        // System is initialized. Track Frame.
        bool bOK;
        if(mState==OK){
            bOK = TrackWithMotionModel();
        }
        

        mLastFrame = Frame(mCurrentFrame);

    }
}

/**
 * @brief 立体视觉相机初始化(与单目相机不同)
 * 1. 设置初始位姿(有IMU时初始位姿不为零)
 * 2. 创建关键帧并插入地图集 (TODO)
 * 3. 为当前帧创建地图点 (TODO)
 */
void Tracking::StereoInitialization()
{
    std::cout << "StereoInitialization:  " << mCurrentFrame.N << std::endl;
    if(mCurrentFrame.N>500)
    {
        std::cout << "StereoInitializing..." << std::endl;
        // if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
        // {
        //     if (!mCurrentFrame.mpImuPreintegrated || !mLastFrame.mpImuPreintegrated)
        //     {
        //         cout << "not IMU meas" << endl;
        //         return;
        //     }

        //     if (!mFastInit && (mCurrentFrame.mpImuPreintegratedFrame->avgA-mLastFrame.mpImuPreintegratedFrame->avgA).norm()<0.5)
        //     {
        //         cout << "not enough acceleration" << endl;
        //         return;
        //     }

        //     if(mpImuPreintegratedFromLastKF)
        //         delete mpImuPreintegratedFromLastKF;

        //     mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
        //     mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
        // }

        // // Set Frame pose to the origin (In case of inertial SLAM to imu)
        // if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
        // {
        //     Eigen::Matrix3f Rwb0 = mCurrentFrame.mImuCalib.mTcb.rotationMatrix();
        //     Eigen::Vector3f twb0 = mCurrentFrame.mImuCalib.mTcb.translation();
        //     Eigen::Vector3f Vwb0;
        //     Vwb0.setZero();
        //     mCurrentFrame.SetImuPoseVelocity(Rwb0, twb0, Vwb0);
        // }
        // else
            mCurrentFrame.SetPose(Sophus::SE3f());

        // // Create KeyFrame
        // KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);

        // // Insert KeyFrame in the map
        // mpAtlas->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        if(!mpCamera2){ // RGBD相机
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                float z = mCurrentFrame.mvDepth[i];
                if(z>0)
                {
                    Eigen::Vector3f x3D;
                    mCurrentFrame.UnprojectStereo(i, x3D);
                    // 这里是通过关键帧生成的地图点
                    // MapPoint* pNewMP = new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());
                    // pNewMP->AddObservation(pKFini,i);
                    // pKFini->AddMapPoint(pNewMP,i);
                    // pNewMP->ComputeDistinctiveDescriptors();
                    // pNewMP->UpdateNormalAndDepth();
                    // mpAtlas->AddMapPoint(pNewMP);
                    // 改成通过帧生成地图点
                    MapPoint* pNewMP = new MapPoint(x3D, &mCurrentFrame, i);
                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    // std::cout << "pos ..." << std::endl;
                    // Eigen::Vector3f pos = pNewMP->GetWorldPos();
                    // std::cout <<  pos(0) << std::endl;
                }
            }
        } 
        // else{ // 双目相机
        //     for(int i = 0; i < mCurrentFrame.Nleft; i++){
        //         int rightIndex = mCurrentFrame.mvLeftToRightMatch[i];
        //         if(rightIndex != -1){
        //             Eigen::Vector3f x3D = mCurrentFrame.mvStereo3Dpoints[i];

        //             MapPoint* pNewMP = new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());

        //             pNewMP->AddObservation(pKFini,i);
        //             pNewMP->AddObservation(pKFini,rightIndex + mCurrentFrame.Nleft);

        //             pKFini->AddMapPoint(pNewMP,i);
        //             pKFini->AddMapPoint(pNewMP,rightIndex + mCurrentFrame.Nleft);

        //             pNewMP->ComputeDistinctiveDescriptors();
        //             pNewMP->UpdateNormalAndDepth();
        //             mpAtlas->AddMapPoint(pNewMP);

        //             mCurrentFrame.mvpMapPoints[i]=pNewMP;
        //             mCurrentFrame.mvpMapPoints[rightIndex + mCurrentFrame.Nleft]=pNewMP;
        //         }
        //     }
        // }

        // Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points", Verbose::VERBOSITY_QUIET);

        //cout << "Active map: " << mpAtlas->GetCurrentMap()->GetId() << endl;

        // mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);
        // mnLastKeyFrameId = mCurrentFrame.mnId;
        // mpLastKeyFrame = pKFini;
        //mnLastRelocFrameId = mCurrentFrame.mnId;

        // mvpLocalKeyFrames.push_back(pKFini);
        // mvpLocalMapPoints=mpAtlas->GetAllMapPoints();
        // mpReferenceKF = pKFini;
        // mCurrentFrame.mpReferenceKF = pKFini;

        // mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

        // mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

        // mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

        mState=OK;
    }
    
}


// void Tracking::MonocularInitialization()
// {

//     if(!mbReadyToInitializate)
//     {
//         // Set Reference Frame
//         if(mCurrentFrame.mvKeys.size()>100)
//         {

//             mInitialFrame = Frame(mCurrentFrame);
//             mLastFrame = Frame(mCurrentFrame);
//             mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
//             for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
//                 mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

//             fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

//             if (mSensor == System::IMU_MONOCULAR)
//             {
//                 if(mpImuPreintegratedFromLastKF)
//                 {
//                     delete mpImuPreintegratedFromLastKF;
//                 }
//                 mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
//                 mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;

//             }

//             mbReadyToInitializate = true;

//             return;
//         }
//     }
//     else
//     {
//         if (((int)mCurrentFrame.mvKeys.size()<=100)||((mSensor == System::IMU_MONOCULAR)&&(mLastFrame.mTimeStamp-mInitialFrame.mTimeStamp>1.0)))
//         {
//             mbReadyToInitializate = false;

//             return;
//         }

//         // Find correspondences
//         ORBmatcher matcher(0.9,true);
//         int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

//         // Check if there are enough correspondences
//         if(nmatches<100)
//         {
//             mbReadyToInitializate = false;
//             return;
//         }

//         Sophus::SE3f Tcw;
//         vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

//         if(mpCamera->ReconstructWithTwoViews(mInitialFrame.mvKeysUn,mCurrentFrame.mvKeysUn,mvIniMatches,Tcw,mvIniP3D,vbTriangulated))
//         {
//             for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
//             {
//                 if(mvIniMatches[i]>=0 && !vbTriangulated[i])
//                 {
//                     mvIniMatches[i]=-1;
//                     nmatches--;
//                 }
//             }

//             // Set Frame Poses
//             mInitialFrame.SetPose(Sophus::SE3f());
//             mCurrentFrame.SetPose(Tcw);

//             CreateInitialMapMonocular();
//         }
//     }
// }



// void Tracking::CreateInitialMapMonocular()
// {
//     // Create KeyFrames
//     KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);
//     KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);

//     if(mSensor == System::IMU_MONOCULAR)
//         pKFini->mpImuPreintegrated = (IMU::Preintegrated*)(NULL);


//     pKFini->ComputeBoW();
//     pKFcur->ComputeBoW();

//     // Insert KFs in the map
//     mpAtlas->AddKeyFrame(pKFini);
//     mpAtlas->AddKeyFrame(pKFcur);

//     for(size_t i=0; i<mvIniMatches.size();i++)
//     {
//         if(mvIniMatches[i]<0)
//             continue;

//         //Create MapPoint.
//         Eigen::Vector3f worldPos;
//         worldPos << mvIniP3D[i].x, mvIniP3D[i].y, mvIniP3D[i].z;
//         MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpAtlas->GetCurrentMap());

//         pKFini->AddMapPoint(pMP,i);
//         pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

//         pMP->AddObservation(pKFini,i);
//         pMP->AddObservation(pKFcur,mvIniMatches[i]);

//         pMP->ComputeDistinctiveDescriptors();
//         pMP->UpdateNormalAndDepth();

//         //Fill Current Frame structure
//         mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
//         mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

//         //Add to Map
//         mpAtlas->AddMapPoint(pMP);
//     }


//     // Update Connections
//     pKFini->UpdateConnections();
//     pKFcur->UpdateConnections();

//     std::set<MapPoint*> sMPs;
//     sMPs = pKFini->GetMapPoints();

//     // Bundle Adjustment
//     Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points", Verbose::VERBOSITY_QUIET);
//     Optimizer::GlobalBundleAdjustemnt(mpAtlas->GetCurrentMap(),20);

//     float medianDepth = pKFini->ComputeSceneMedianDepth(2);
//     float invMedianDepth;
//     if(mSensor == System::IMU_MONOCULAR)
//         invMedianDepth = 4.0f/medianDepth; // 4.0f
//     else
//         invMedianDepth = 1.0f/medianDepth;

//     if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<50) // TODO Check, originally 100 tracks
//     {
//         Verbose::PrintMess("Wrong initialization, reseting...", Verbose::VERBOSITY_QUIET);
//         mpSystem->ResetActiveMap();
//         return;
//     }

//     // Scale initial baseline
//     Sophus::SE3f Tc2w = pKFcur->GetPose();
//     Tc2w.translation() *= invMedianDepth;
//     pKFcur->SetPose(Tc2w);

//     // Scale points
//     vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
//     for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
//     {
//         if(vpAllMapPoints[iMP])
//         {
//             MapPoint* pMP = vpAllMapPoints[iMP];
//             pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
//             pMP->UpdateNormalAndDepth();
//         }
//     }

//     if (mSensor == System::IMU_MONOCULAR)
//     {
//         pKFcur->mPrevKF = pKFini;
//         pKFini->mNextKF = pKFcur;
//         pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

//         mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKFcur->mpImuPreintegrated->GetUpdatedBias(),pKFcur->mImuCalib);
//     }


//     mpLocalMapper->InsertKeyFrame(pKFini);
//     mpLocalMapper->InsertKeyFrame(pKFcur);
//     mpLocalMapper->mFirstTs=pKFcur->mTimeStamp;

//     mCurrentFrame.SetPose(pKFcur->GetPose());
//     mnLastKeyFrameId=mCurrentFrame.mnId;
//     mpLastKeyFrame = pKFcur;
//     //mnLastRelocFrameId = mInitialFrame.mnId;

//     mvpLocalKeyFrames.push_back(pKFcur);
//     mvpLocalKeyFrames.push_back(pKFini);
//     mvpLocalMapPoints=mpAtlas->GetAllMapPoints();
//     mpReferenceKF = pKFcur;
//     mCurrentFrame.mpReferenceKF = pKFcur;

//     // Compute here initial velocity
//     vector<KeyFrame*> vKFs = mpAtlas->GetAllKeyFrames();

//     Sophus::SE3f deltaT = vKFs.back()->GetPose() * vKFs.front()->GetPoseInverse();
//     mbVelocity = false;
//     Eigen::Vector3f phi = deltaT.so3().log();

//     double aux = (mCurrentFrame.mTimeStamp-mLastFrame.mTimeStamp)/(mCurrentFrame.mTimeStamp-mInitialFrame.mTimeStamp);
//     phi *= aux;

//     mLastFrame = Frame(mCurrentFrame);

//     mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

//     mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

//     mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

//     mState=OK;

//     initID = pKFcur->mnId;
// }


// void Tracking::CreateMapInAtlas()
// {
//     mnLastInitFrameId = mCurrentFrame.mnId;
//     mpAtlas->CreateNewMap();
//     if (mSensor==System::IMU_STEREO || mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_RGBD)
//         mpAtlas->SetInertialSensor();
//     mbSetInit=false;

//     mnInitialFrameId = mCurrentFrame.mnId+1;
//     mState = NO_IMAGES_YET;

//     // Restart the variable with information about the last KF
//     mbVelocity = false;
//     //mnLastRelocFrameId = mnLastInitFrameId; // The last relocation KF_id is the current id, because it is the new starting point for new map
//     Verbose::PrintMess("First frame id in map: " + to_string(mnLastInitFrameId+1), Verbose::VERBOSITY_NORMAL);
//     mbVO = false; // Init value for know if there are enough MapPoints in the last KF
//     if(mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
//     {
//         mbReadyToInitializate = false;
//     }

//     if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && mpImuPreintegratedFromLastKF)
//     {
//         delete mpImuPreintegratedFromLastKF;
//         mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
//     }

//     if(mpLastKeyFrame)
//         mpLastKeyFrame = static_cast<KeyFrame*>(NULL);

//     if(mpReferenceKF)
//         mpReferenceKF = static_cast<KeyFrame*>(NULL);

//     mLastFrame = Frame();
//     mCurrentFrame = Frame();
//     mvIniMatches.clear();

//     mbCreatedMap = true;
// }

// void Tracking::CheckReplacedInLastFrame()
// {
//     for(int i =0; i<mLastFrame.N; i++)
//     {
//         MapPoint* pMP = mLastFrame.mvpMapPoints[i];

//         if(pMP)
//         {
//             MapPoint* pRep = pMP->GetReplaced();
//             if(pRep)
//             {
//                 mLastFrame.mvpMapPoints[i] = pRep;
//             }
//         }
//     }
// }


// bool Tracking::TrackReferenceKeyFrame()
// {
//     // Compute Bag of Words vector
//     mCurrentFrame.ComputeBoW();

//     // We perform first an ORB matching with the reference keyframe
//     // If enough matches are found we setup a PnP solver
//     ORBmatcher matcher(0.7,true);
//     vector<MapPoint*> vpMapPointMatches;

//     int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

//     if(nmatches<15)
//     {
//         cout << "TRACK_REF_KF: Less than 15 matches!!\n";
//         return false;
//     }

//     mCurrentFrame.mvpMapPoints = vpMapPointMatches;
//     mCurrentFrame.SetPose(mLastFrame.GetPose());

//     //mCurrentFrame.PrintPointDistribution();


//     // cout << " TrackReferenceKeyFrame mLastFrame.mTcw:  " << mLastFrame.mTcw << endl;
//     Optimizer::PoseOptimization(&mCurrentFrame);

//     // Discard outliers
//     int nmatchesMap = 0;
//     for(int i =0; i<mCurrentFrame.N; i++)
//     {
//         //if(i >= mCurrentFrame.Nleft) break;
//         if(mCurrentFrame.mvpMapPoints[i])
//         {
//             if(mCurrentFrame.mvbOutlier[i])
//             {
//                 MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

//                 mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
//                 mCurrentFrame.mvbOutlier[i]=false;
//                 if(i < mCurrentFrame.Nleft){
//                     pMP->mbTrackInView = false;
//                 }
//                 else{
//                     pMP->mbTrackInViewR = false;
//                 }
//                 pMP->mbTrackInView = false;
//                 pMP->mnLastFrameSeen = mCurrentFrame.mnId;
//                 nmatches--;
//             }
//             else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
//                 nmatchesMap++;
//         }
//     }

//     if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
//         return true;
//     else
//         return nmatchesMap>=10;
// }

/**
 * @brief 更新上一帧的相关信息,我们这里给上一帧生成地图点
 * 1. 位姿 Pose
 * 2. 特征点
 */
void Tracking::UpdateLastFrame()
{
    // // Update pose according to reference keyframe
    // KeyFrame* pRef = mLastFrame.mpReferenceKF;
    // // 当前帧到参考关键帧的transform
    // Sophus::SE3f Tlr = mlRelativeFramePoses.back();
    // // 上一帧的Pose Tc1c0 * Tc0w = Tc1w
    // mLastFrame.SetPose(Tlr * pRef->GetPose());
    
    // if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR || !mbOnlyTracking)
    //     return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    //         深度值 特征点id
    vector<pair<float,int> > vDepthIdx;
    // 左目相机提取出的特征点数量
    const int Nfeat = mCurrentFrame.N;
    vDepthIdx.reserve(Nfeat);
    for(int i=0; i<Nfeat;i++)
    {
        float z = mCurrentFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    // 按照深度值排序
    sort(vDepthIdx.begin(),vDepthIdx.end());

    // 找到100个比较近的特征点
    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        // MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        // if(!pMP)
            bCreateNew = true;
        // else if(pMP->Observations()<1)
        //     bCreateNew = true;

        // 是否要创建新的MapPoint
        if(bCreateNew)
        {
            Eigen::Vector3f x3D;
            // if(mLastFrame.Nleft == -1){
                mCurrentFrame.UnprojectStereo(i, x3D);
            // }
            // else{
            //     x3D = mLastFrame.UnprojectStereoFishEye(i);
            // }
            // x3D设置为worldpos
            MapPoint* pNewMP = new MapPoint(x3D,&mCurrentFrame,i);
            mCurrentFrame.mvpMapPoints[i]=pNewMP;
            std::cout << "pos ..." << std::endl;
            // Eigen::Vector3f pos = pNewMP->GetWorldPos();
            // std::cout <<  pos(0) << std::endl;

            // mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }
        // 终止条件
        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;

    }
}

/**
 * @brief 
 * 1. 更新上一帧信息(补充地图点)
 * 2. 设置当前帧位姿(恒速模型)
 * 3. 特征点匹配(通过重投影缩小匹配范围，快速找到匹配特征点对)
 * 4. 位置优化(通过上一步匹配得到匹配成功的地图点，该地图点在当前帧里有对应的特征点像素坐标，该地图点重投影也可以得到一个像素坐标，这两个像素坐标之间的距离就是误差)
 *    优化图: 
 *          顶点为相机位姿SE3
 *          误差边为上边求得的那个冲投影误差
 *    (相当于根据匹配到的特征点调整相机位姿)
 * PS: 1. 之后当前帧里的地图点只有上一帧投影过来，并且成功匹配的
 */
bool Tracking::TrackWithMotionModel()
{
    // ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    mCurrentFrame.SetPose(Sophus::SE3f());

    UpdateLastFrame();
    
    // // 根据IMU预测当前帧位姿
    // if (mpAtlas->isImuInitialized() && (mCurrentFrame.mnId>mnLastRelocFrameId+mnFramesToResetIMU))
    // {
    //     // Predict state with IMU if it is initialized and it doesnt need reset
    //     PredictStateIMU();
    //     return true;
    // }
    // // 跟踪恒速模型预测当前帧位姿
    // else{
        // mCurrentFrame.SetPose(mVelocity * mLastFrame.GetPose());
    // }



    // 初始化地图点(地图点就是特征点的3D坐标)
    // fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;

    // if(mSensor==System::STEREO)
    //     th=7;
    // else
    th=15;

    // // 特征点匹配(根据前一帧的MapPoints重投影，然后在投影点的一定范围内进行ORB特征匹配，生成当前帧的MapPoints)
    // int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR);

    // // If few matches, uses a wider window search
    // if(nmatches<20)
    // {
    //     Verbose::PrintMess("Not enough matches, wider window search!!", Verbose::VERBOSITY_NORMAL);
    //     fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    //     nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR);
    //     Verbose::PrintMess("Matches with wider search: " + to_string(nmatches), Verbose::VERBOSITY_NORMAL);

    // }

    // if(nmatches<20)
    // {
    //     Verbose::PrintMess("Not enough matches!!", Verbose::VERBOSITY_NORMAL);
    //     if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
    //         return true;
    //     else
    //         return false;
    // }

    // // Optimize frame pose with all matches
    // Optimizer::PoseOptimization(&mCurrentFrame);

    // // 优化时会有内点、外点的区分，这里把被判断为外点的那些点都剔除
    // // Discard outliers
    // int nmatchesMap = 0;
    // for(int i =0; i<mCurrentFrame.N; i++)
    // {
    //     if(mCurrentFrame.mvpMapPoints[i])
    //     {
    //         if(mCurrentFrame.mvbOutlier[i]) // 如果是外点
    //         {
    //             MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

    //             mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
    //             mCurrentFrame.mvbOutlier[i]=false;
    //             if(i < mCurrentFrame.Nleft){
    //                 pMP->mbTrackInView = false;
    //             }
    //             else{
    //                 pMP->mbTrackInViewR = false;
    //             }
    //             pMP->mnLastFrameSeen = mCurrentFrame.mnId;
    //             nmatches--;
    //         }
    //         else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
    //             nmatchesMap++;
    //     }
    // }

    // if(mbOnlyTracking)
    // {
    //     mbVO = nmatchesMap<10;
    //     return nmatches>20;
    // }

    // if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
    //     return true;
    // else
    //     return nmatchesMap>=10;
    return true;
}

// /**
//  * @brief 跟踪局部地图
//  * 1. 更新局部地图，找到有共视关系的地图点
//  * 2. 更新当前帧的地图点(为局部地图里的地图点)
//  * 3. 再一次优化当前帧位姿(完全一样的方法，只不过是地图点得到了更新)
//  * 4. 更新地图点数据
//  *      遍历当前帧的
//  */
// bool Tracking::TrackLocalMap()
// {

//     // We have an estimation of the camera pose and some map points tracked in the frame.
//     // We retrieve the local map and try to find matches to points in the local map.
//     mTrackedFr++;

//     /**     1       **/
//     UpdateLocalMap();

//     /**     2       **/
//     SearchLocalPoints();

//     // TOO check outliers before PO
//     int aux1 = 0, aux2=0;
//     for(int i=0; i<mCurrentFrame.N; i++)
//         if( mCurrentFrame.mvpMapPoints[i])
//         {
//             aux1++;
//             if(mCurrentFrame.mvbOutlier[i])
//                 aux2++;
//         }
    
//     /**     3       **/
//     // 这里我们没有使用IMU
//     int inliers;
//     if (!mpAtlas->isImuInitialized())
//         Optimizer::PoseOptimization(&mCurrentFrame);
//     else
//     {
//         if(mCurrentFrame.mnId<=mnLastRelocFrameId+mnFramesToResetIMU)
//         {
//             Verbose::PrintMess("TLM: PoseOptimization ", Verbose::VERBOSITY_DEBUG);
//             Optimizer::PoseOptimization(&mCurrentFrame);
//         }
//         else
//         {
//             // if(!mbMapUpdated && mState == OK) //  && (mnMatchesInliers>30))
//             if(!mbMapUpdated) //  && (mnMatchesInliers>30))
//             {
//                 Verbose::PrintMess("TLM: PoseInertialOptimizationLastFrame ", Verbose::VERBOSITY_DEBUG);
//                 inliers = Optimizer::PoseInertialOptimizationLastFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
//             }
//             else
//             {
//                 Verbose::PrintMess("TLM: PoseInertialOptimizationLastKeyFrame ", Verbose::VERBOSITY_DEBUG);
//                 inliers = Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
//             }
//         }
//     }

//     aux1 = 0, aux2 = 0;
//     for(int i=0; i<mCurrentFrame.N; i++)
//         if( mCurrentFrame.mvpMapPoints[i])
//         {
//             aux1++;
//             if(mCurrentFrame.mvbOutlier[i])
//                 aux2++;
//         }

//     mnMatchesInliers = 0;

//     /**     4       **/
//     // Update MapPoints Statistics
//     for(int i=0; i<mCurrentFrame.N; i++)
//     {
//         if(mCurrentFrame.mvpMapPoints[i])
//         {
//             if(!mCurrentFrame.mvbOutlier[i]) // 只有优化时的内点才算真正观测到了
//             {
//                 mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
//                 if(!mbOnlyTracking)
//                 {
//                     if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
//                         mnMatchesInliers++;
//                 }
//                 else
//                     mnMatchesInliers++;
//             }
//             else if(mSensor==System::STEREO)
//                 mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
//         }
//     }

//     // Decide if the tracking was succesful
//     // More restrictive if there was a relocalization recently
//     mpLocalMapper->mnMatchesInliers=mnMatchesInliers;
//     if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
//         return false;

//     if((mnMatchesInliers>10)&&(mState==RECENTLY_LOST))
//         return true;


//     if (mSensor == System::IMU_MONOCULAR)
//     {
//         if((mnMatchesInliers<15 && mpAtlas->isImuInitialized())||(mnMatchesInliers<50 && !mpAtlas->isImuInitialized()))
//         {
//             return false;
//         }
//         else
//             return true;
//     }
//     else if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
//     {
//         if(mnMatchesInliers<15)
//         {
//             return false;
//         }
//         else
//             return true;
//     }
//     else
//     {
//         if(mnMatchesInliers<30)
//             return false;
//         else
//             return true;
//     }
// }

// bool Tracking::NeedNewKeyFrame()
// {
//     if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && !mpAtlas->GetCurrentMap()->isImuInitialized())
//     {
//         if (mSensor == System::IMU_MONOCULAR && (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.25)
//             return true;
//         else if ((mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.25)
//             return true;
//         else
//             return false;
//     }

//     if(mbOnlyTracking)
//         return false;

//     // If Local Mapping is freezed by a Loop Closure do not insert keyframes
//     if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested()) {
//         /*if(mSensor == System::MONOCULAR)
//         {
//             std::cout << "NeedNewKeyFrame: localmap stopped" << std::endl;
//         }*/
//         return false;
//     }

//     const int nKFs = mpAtlas->KeyFramesInMap();

//     // Do not insert keyframes if not enough frames have passed from last relocalisation
//     if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
//     {
//         return false;
//     }

//     // Tracked MapPoints in the reference keyframe
//     int nMinObs = 3;
//     if(nKFs<=2)
//         nMinObs=2;
//     int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

//     // Local Mapping accept keyframes?
//     bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

//     // Check how many "close" points are being tracked and how many could be potentially created.
//     int nNonTrackedClose = 0;
//     int nTrackedClose= 0;

//     if(mSensor!=System::MONOCULAR && mSensor!=System::IMU_MONOCULAR)
//     {
//         int N = (mCurrentFrame.Nleft == -1) ? mCurrentFrame.N : mCurrentFrame.Nleft;
//         for(int i =0; i<N; i++)
//         {
//             if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
//             {
//                 if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
//                     nTrackedClose++;
//                 else
//                     nNonTrackedClose++;

//             }
//         }
//         //Verbose::PrintMess("[NEEDNEWKF]-> closed points: " + to_string(nTrackedClose) + "; non tracked closed points: " + to_string(nNonTrackedClose), Verbose::VERBOSITY_NORMAL);// Verbose::VERBOSITY_DEBUG);
//     }

//     bool bNeedToInsertClose;
//     bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

//     // Thresholds
//     float thRefRatio = 0.75f;
//     if(nKFs<2)
//         thRefRatio = 0.4f;

//     /*int nClosedPoints = nTrackedClose + nNonTrackedClose;
//     const int thStereoClosedPoints = 15;
//     if(nClosedPoints < thStereoClosedPoints && (mSensor==System::STEREO || mSensor==System::IMU_STEREO))
//     {
//         //Pseudo-monocular, there are not enough close points to be confident about the stereo observations.
//         thRefRatio = 0.9f;
//     }*/

//     if(mSensor==System::MONOCULAR)
//         thRefRatio = 0.9f;

//     if(mpCamera2) thRefRatio = 0.75f;

//     if(mSensor==System::IMU_MONOCULAR)
//     {
//         if(mnMatchesInliers>350) // Points tracked from the local map
//             thRefRatio = 0.75f;
//         else
//             thRefRatio = 0.90f;
//     }

//     // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
//     const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
//     // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
//     const bool c1b = ((mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames) && bLocalMappingIdle); //mpLocalMapper->KeyframesInQueue() < 2);
//     //Condition 1c: tracking is weak
//     const bool c1c = mSensor!=System::MONOCULAR && mSensor!=System::IMU_MONOCULAR && mSensor!=System::IMU_STEREO && mSensor!=System::IMU_RGBD && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
//     // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
//     const bool c2 = (((mnMatchesInliers<nRefMatches*thRefRatio || bNeedToInsertClose)) && mnMatchesInliers>15);

//     //std::cout << "NeedNewKF: c1a=" << c1a << "; c1b=" << c1b << "; c1c=" << c1c << "; c2=" << c2 << std::endl;
//     // Temporal condition for Inertial cases
//     bool c3 = false;
//     if(mpLastKeyFrame)
//     {
//         if (mSensor==System::IMU_MONOCULAR)
//         {
//             if ((mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.5)
//                 c3 = true;
//         }
//         else if (mSensor==System::IMU_STEREO || mSensor == System::IMU_RGBD)
//         {
//             if ((mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.5)
//                 c3 = true;
//         }
//     }

//     bool c4 = false;
//     if ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && (mSensor == System::IMU_MONOCULAR)) // MODIFICATION_2, originally ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && ((mSensor == System::IMU_MONOCULAR)))
//         c4=true;
//     else
//         c4=false;

//     if(((c1a||c1b||c1c) && c2)||c3 ||c4)
//     {
//         // If the mapping accepts keyframes, insert keyframe.
//         // Otherwise send a signal to interrupt BA
//         if(bLocalMappingIdle || mpLocalMapper->IsInitializing())
//         {
//             return true;
//         }
//         else
//         {
//             mpLocalMapper->InterruptBA();
//             if(mSensor!=System::MONOCULAR  && mSensor!=System::IMU_MONOCULAR)
//             {
//                 if(mpLocalMapper->KeyframesInQueue()<3)
//                     return true;
//                 else
//                     return false;
//             }
//             else
//             {
//                 //std::cout << "NeedNewKeyFrame: localmap is busy" << std::endl;
//                 return false;
//             }
//         }
//     }
//     else
//         return false;
// }

// /**
//  * @brief 创建关键帧
//  * 1. 复制当前帧数据到关键帧
//  * 2. 连接前一个关键帧
//  * 3. 对于双目或rgbd摄像头，为当前帧和当前关键帧生成新的MapPoints
//  */
// void Tracking::CreateNewKeyFrame()
// {
//     if(mpLocalMapper->IsInitializing() && !mpAtlas->isImuInitialized())
//         return;

//     if(!mpLocalMapper->SetNotStop(true))
//         return;

//     KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);

//     if(mpAtlas->isImuInitialized()) //  || mpLocalMapper->IsInitializing())
//         pKF->bImu = true;

//     pKF->SetNewBias(mCurrentFrame.mImuBias);
//     mpReferenceKF = pKF;
//     mCurrentFrame.mpReferenceKF = pKF;

//     /** 2.连接KeyFrame **/
//     if(mpLastKeyFrame)
//     {
//         pKF->mPrevKF = mpLastKeyFrame;
//         mpLastKeyFrame->mNextKF = pKF;
//     }
//     else
//         Verbose::PrintMess("No last KF in KF creation!!", Verbose::VERBOSITY_NORMAL);

//     // Reset preintegration from last KF (Create new object)
//     if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
//     {
//         mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKF->GetImuBias(),pKF->mImuCalib);
//     }

//     if(mSensor!=System::MONOCULAR && mSensor != System::IMU_MONOCULAR) // TODO check if incluide imu_stereo
//     {
//         mCurrentFrame.UpdatePoseMatrices();
//         // cout << "create new MPs" << endl;
//         // We sort points by the measured depth by the stereo/RGBD sensor.
//         // We create all those MapPoints whose depth < mThDepth.
//         // If there are less than 100 close points we create the 100 closest.
//         int maxPoint = 100;
//         if(mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
//             maxPoint = 100;
//         // Step 3.1：得到当前帧有深度值的特征点（不一定是地图点）
//         vector<pair<float,int> > vDepthIdx;
//         int N = (mCurrentFrame.Nleft != -1) ? mCurrentFrame.Nleft : mCurrentFrame.N;
//         vDepthIdx.reserve(mCurrentFrame.N);
//         for(int i=0; i<N; i++)
//         {
//             float z = mCurrentFrame.mvDepth[i];// 每个特征点都对应一个深度
//             if(z>0)
//             {
//                 vDepthIdx.push_back(make_pair(z,i));
//             }
//         }

//         if(!vDepthIdx.empty())
//         {
//             // Step 3.2：按照深度从小到大排序
//             sort(vDepthIdx.begin(),vDepthIdx.end());
//             // Step 3.3：从中找出不是地图点的生成临时地图点 
//             int nPoints = 0;
//             for(size_t j=0; j<vDepthIdx.size();j++)
//             {
//                 int i = vDepthIdx[j].second;

//                 bool bCreateNew = false;
//                 // 如果这个点没有对应的地图点,或者创建后就没有被观测到,那么就生成一个临时的地图点
//                 MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
//                 if(!pMP)
//                     bCreateNew = true;
//                 else if(pMP->Observations()<1)
//                 {
//                     bCreateNew = true;
//                     mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
//                 }
//                 // 如果需要就新建地图点，这里的地图点不是临时的，是全局地图中新建地图点，用于跟踪
//                 if(bCreateNew)
//                 {
//                     Eigen::Vector3f x3D;

//                     if(mCurrentFrame.Nleft == -1){
//                         mCurrentFrame.UnprojectStereo(i, x3D);
//                     }
//                     else{
//                         x3D = mCurrentFrame.UnprojectStereoFishEye(i);
//                     }

//                     MapPoint* pNewMP = new MapPoint(x3D,pKF,mpAtlas->GetCurrentMap());
//                     pNewMP->AddObservation(pKF,i);

//                     //Check if it is a stereo observation in order to not
//                     //duplicate mappoints
//                     if(mCurrentFrame.Nleft != -1 && mCurrentFrame.mvLeftToRightMatch[i] >= 0){
//                         mCurrentFrame.mvpMapPoints[mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]]=pNewMP;
//                         pNewMP->AddObservation(pKF,mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
//                         pKF->AddMapPoint(pNewMP,mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
//                     }

//                     pKF->AddMapPoint(pNewMP,i);
//                     pNewMP->ComputeDistinctiveDescriptors();
//                     pNewMP->UpdateNormalAndDepth();
//                     mpAtlas->AddMapPoint(pNewMP);

//                     mCurrentFrame.mvpMapPoints[i]=pNewMP;
//                     nPoints++;
//                 }
//                 else
//                 {
//                     nPoints++;
//                 }
//                 // Step 3.4：停止新建地图点必须同时满足以下条件：
//                 // 1、当前的点的深度已经超过了设定的深度阈值（35倍基线）
//                 // 2、nPoints已经超过100个点，说明距离比较远了，可能不准确，停掉退出
//                 if(vDepthIdx[j].first>mThDepth && nPoints>maxPoint)
//                 {
//                     break;
//                 }
//             }
//             //Verbose::PrintMess("new mps for stereo KF: " + to_string(nPoints), Verbose::VERBOSITY_NORMAL);
//         }
//     }

//     // Step 4：插入关键帧
//     // 关键帧插入到列表 mlNewKeyFrames中，等待local mapping线程临幸
//     mpLocalMapper->InsertKeyFrame(pKF);
//     // 插入好了，允许局部建图停止
//     mpLocalMapper->SetNotStop(false);
//     // 当前帧成为新的关键帧，更新
//     mnLastKeyFrameId = mCurrentFrame.mnId;
//     mpLastKeyFrame = pKF;
// }

// /**
//  * @brief 
//  * 1. 遍历地图点，判断是否在当前帧视野内
//  * 2. 如果在，则把该地图点投影到当前帧里，并找到最匹配的特征点，更新这个特征点对应的地图点
//  * 1. 遍历当前帧的地图点(这些地图点是从上一帧重投影过来的，是经过特征匹配的)
//  *    IncreaseVisible(): 因为这些点可以被当前帧观测到
//  * 2. 遍历 mvpLocalMapPoints (这里不包括当前帧的地图点，除了上一帧投影过来的地图点以外还有其他的局部地图点可能投影到当前帧里)
//  *    IncreaseVisible(): 这些点也被当前帧观测到了
//  * 3. 之后会改变当前帧的地图点数量(增加哪些LocalMapPoints投影过来的)
//  */
// void Tracking::SearchLocalPoints()
// {
//     // Do not search map points already matched
//     for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
//     {
//         MapPoint* pMP = *vit;
//         if(pMP)
//         {
//             if(pMP->isBad())
//             {
//                 *vit = static_cast<MapPoint*>(NULL);
//             }
//             else
//             {
//                 pMP->IncreaseVisible();
//                 pMP->mnLastFrameSeen = mCurrentFrame.mnId;
//                 pMP->mbTrackInView = false;
//                 pMP->mbTrackInViewR = false;
//             }
//         }
//     }

//     int nToMatch=0;

//     // Project points in frame and check its visibility
//     for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
//     {
//         MapPoint* pMP = *vit;

//         if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
//             continue;
//         if(pMP->isBad())
//             continue;
//         // Project (this fills MapPoint variables for matching)
//         if(mCurrentFrame.isInFrustum(pMP,0.5))
//         {
//             pMP->IncreaseVisible();
//             nToMatch++;
//         }
//         if(pMP->mbTrackInView)
//         {
//             mCurrentFrame.mmProjectPoints[pMP->mnId] = cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY);
//         }
//     }

//     if(nToMatch>0)
//     {
//         ORBmatcher matcher(0.8);
//         int th = 1;
//         if(mSensor==System::RGBD || mSensor==System::IMU_RGBD)
//             th=3;
//         if(mpAtlas->isImuInitialized())
//         {
//             if(mpAtlas->GetCurrentMap()->GetIniertialBA2())
//                 th=2;
//             else
//                 th=6;
//         }
//         else if(!mpAtlas->isImuInitialized() && (mSensor==System::IMU_MONOCULAR || mSensor==System::IMU_STEREO || mSensor == System::IMU_RGBD))
//         {
//             th=10;
//         }

//         // If the camera has been relocalised recently, perform a coarser search
//         if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
//             th=5;

//         if(mState==LOST || mState==RECENTLY_LOST) // Lost for less than 1 second
//             th=15; // 15

//         int matches = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th, mpLocalMapper->mbFarPoints, mpLocalMapper->mThFarPoints);
//     }
// }

// void Tracking::UpdateLocalMap()
// {
//     // This is for visualization
//     mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

//     // Update
//     UpdateLocalKeyFrames();
//     UpdateLocalPoints();
// }

// /**
//  * @brief 生成 mvpLocalMapPoints , 临边存放的是这些局部关键帧里的地图点， 但是不包括当前帧里的地图点
//  */
// void Tracking::UpdateLocalPoints()
// {
//     mvpLocalMapPoints.clear();

//     int count_pts = 0;

//     for(vector<KeyFrame*>::const_reverse_iterator itKF=mvpLocalKeyFrames.rbegin(), itEndKF=mvpLocalKeyFrames.rend(); itKF!=itEndKF; ++itKF)
//     {
//         KeyFrame* pKF = *itKF;
//         const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

//         for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
//         {

//             MapPoint* pMP = *itMP;
//             if(!pMP)
//                 continue;
//             if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
//                 continue;
//             if(!pMP->isBad())
//             {
//                 count_pts++;
//                 mvpLocalMapPoints.push_back(pMP);
//                 pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
//             }
//         }
//     }
// }

// /**
//  * @brief 这里生成了 mvpLocalKeyFrames ， 里边存放的是和当前帧有共视关系的关键帧.(共视关系指：看到相同的地图点)
//  * 1. 遍历所有地图点，得到观测到此地图点的关键帧，因为有很多地图点，所以一个关键帧可能观测到很多地图点
//  * 2. 遍历所有挑选出来的关键帧，放到 mvpLocalKeyFrames 里，并找到一个和当前帧共视程度最高的关键帧 pKFmax 
//  * 3. 遍历所有 mvpLocalKeyFrames ， 把其中关键帧的临近关键帧也放到里 (这个临近关键帧是通过最小生成树来维护的)
//  * 4. 把共视程度最高的关键帧 pKFmax 作为当前帧的参考关键帧
//  */
// void Tracking::UpdateLocalKeyFrames()
// {
//     // Each map point vote for the keyframes in which it has been observed
//     //   关键帧  观测到关键点的个数
//     map<KeyFrame*,int> keyframeCounter;
//     if(!mpAtlas->isImuInitialized() || (mCurrentFrame.mnId<mnLastRelocFrameId+2))
//     {
//         // 遍历所有地图点
//         for(int i=0; i<mCurrentFrame.N; i++)
//         {
//             MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
//             if(pMP)
//             {
//                 if(!pMP->isBad())
//                 {
//                     // 这里应该是被观测到的次数，整个点可以被许多关键帧观测到
//                     const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();
//                     for(map<KeyFrame*,tuple<int,int>>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
//                         keyframeCounter[it->first]++;
//                 }
//                 else
//                 {
//                     mCurrentFrame.mvpMapPoints[i]=NULL;
//                 }
//             }
//         }
//     }
//     else
//     {
//         for(int i=0; i<mLastFrame.N; i++)
//         {
//             // Using lastframe since current frame has not matches yet
//             if(mLastFrame.mvpMapPoints[i])
//             {
//                 MapPoint* pMP = mLastFrame.mvpMapPoints[i];
//                 if(!pMP)
//                     continue;
//                 if(!pMP->isBad())
//                 {
//                     const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();
//                     for(map<KeyFrame*,tuple<int,int>>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
//                         keyframeCounter[it->first]++;
//                 }
//                 else
//                 {
//                     // MODIFICATION
//                     mLastFrame.mvpMapPoints[i]=NULL;
//                 }
//             }
//         }
//     }

//     /**  2.收集局部关键帧   **/
//     int max=0;
//     KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

//     mvpLocalKeyFrames.clear();
//     mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

//     // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
//     for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
//     {
//         KeyFrame* pKF = it->first;

//         if(pKF->isBad())
//             continue;

//         if(it->second>max)
//         {
//             max=it->second;
//             pKFmax=pKF;
//         }

//         mvpLocalKeyFrames.push_back(pKF);
//         pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
//     }

//     /**  3.扩充局部关键帧   **/
//     // Include also some not-already-included keyframes that are neighbors to already-included keyframes
//     for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
//     {// 遍历所有局部关键帧
//         // Limit the number of keyframes
//         if(mvpLocalKeyFrames.size()>80) // 80
//             break;

//         KeyFrame* pKF = *itKF;

//         // 拿出这个关键帧的10个临近关键帧，把这些临近的也放到局部关键帧里
//         const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);


//         for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
//         {
//             KeyFrame* pNeighKF = *itNeighKF;
//             if(!pNeighKF->isBad())
//             {
//                 if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
//                 {
//                     mvpLocalKeyFrames.push_back(pNeighKF);
//                     pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
//                     break;
//                 }
//             }
//         }

//         // 拿出这个关键帧的儿子关键帧，也放到局部关键帧里
//         const set<KeyFrame*> spChilds = pKF->GetChilds();
//         for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
//         {
//             KeyFrame* pChildKF = *sit;
//             if(!pChildKF->isBad())
//             {
//                 if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
//                 {
//                     mvpLocalKeyFrames.push_back(pChildKF);
//                     pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
//                     break;
//                 }
//             }
//         }

//         // 拿出这个关键帧的父亲关键帧，也放到局部关键帧里
//         KeyFrame* pParent = pKF->GetParent();
//         if(pParent)
//         {
//             if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
//             {
//                 mvpLocalKeyFrames.push_back(pParent);
//                 pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
//                 break;
//             }
//         }
//     }

//     // Add 10 last temporal KFs (mainly for IMU)
//     if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) &&mvpLocalKeyFrames.size()<80)
//     {
//         KeyFrame* tempKeyFrame = mCurrentFrame.mpLastKeyFrame;

//         const int Nd = 20;
//         for(int i=0; i<Nd; i++){
//             if (!tempKeyFrame)
//                 break;
//             if(tempKeyFrame->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
//             {
//                 mvpLocalKeyFrames.push_back(tempKeyFrame);
//                 tempKeyFrame->mnTrackReferenceForFrame=mCurrentFrame.mnId;
//                 tempKeyFrame=tempKeyFrame->mPrevKF;
//             }
//         }
//     }

//     if(pKFmax)
//     {
//         mpReferenceKF = pKFmax;
//         mCurrentFrame.mpReferenceKF = mpReferenceKF;
//     }
// }

// bool Tracking::Relocalization()
// {
//     Verbose::PrintMess("Starting relocalization", Verbose::VERBOSITY_NORMAL);
//     // Compute Bag of Words Vector
//     mCurrentFrame.ComputeBoW();

//     // Relocalization is performed when tracking is lost
//     // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
//     vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame, mpAtlas->GetCurrentMap());

//     if(vpCandidateKFs.empty()) {
//         Verbose::PrintMess("There are not candidates", Verbose::VERBOSITY_NORMAL);
//         return false;
//     }

//     const int nKFs = vpCandidateKFs.size();

//     // We perform first an ORB matching with each candidate
//     // If enough matches are found we setup a PnP solver
//     ORBmatcher matcher(0.75,true);

//     vector<MLPnPsolver*> vpMLPnPsolvers;
//     vpMLPnPsolvers.resize(nKFs);

//     vector<vector<MapPoint*> > vvpMapPointMatches;
//     vvpMapPointMatches.resize(nKFs);

//     vector<bool> vbDiscarded;
//     vbDiscarded.resize(nKFs);

//     int nCandidates=0;

//     for(int i=0; i<nKFs; i++)
//     {
//         KeyFrame* pKF = vpCandidateKFs[i];
//         if(pKF->isBad())
//             vbDiscarded[i] = true;
//         else
//         {
//             int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
//             if(nmatches<15)
//             {
//                 vbDiscarded[i] = true;
//                 continue;
//             }
//             else
//             {
//                 MLPnPsolver* pSolver = new MLPnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
//                 pSolver->SetRansacParameters(0.99,10,300,6,0.5,5.991);  //This solver needs at least 6 points
//                 vpMLPnPsolvers[i] = pSolver;
//                 nCandidates++;
//             }
//         }
//     }

//     // Alternatively perform some iterations of P4P RANSAC
//     // Until we found a camera pose supported by enough inliers
//     bool bMatch = false;
//     ORBmatcher matcher2(0.9,true);

//     while(nCandidates>0 && !bMatch)
//     {
//         for(int i=0; i<nKFs; i++)
//         {
//             if(vbDiscarded[i])
//                 continue;

//             // Perform 5 Ransac Iterations
//             vector<bool> vbInliers;
//             int nInliers;
//             bool bNoMore;

//             MLPnPsolver* pSolver = vpMLPnPsolvers[i];
//             Eigen::Matrix4f eigTcw;
//             bool bTcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers, eigTcw);

//             // If Ransac reachs max. iterations discard keyframe
//             if(bNoMore)
//             {
//                 vbDiscarded[i]=true;
//                 nCandidates--;
//             }

//             // If a Camera Pose is computed, optimize
//             if(bTcw)
//             {
//                 Sophus::SE3f Tcw(eigTcw);
//                 mCurrentFrame.SetPose(Tcw);
//                 // Tcw.copyTo(mCurrentFrame.mTcw);

//                 set<MapPoint*> sFound;

//                 const int np = vbInliers.size();

//                 for(int j=0; j<np; j++)
//                 {
//                     if(vbInliers[j])
//                     {
//                         mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
//                         sFound.insert(vvpMapPointMatches[i][j]);
//                     }
//                     else
//                         mCurrentFrame.mvpMapPoints[j]=NULL;
//                 }

//                 int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

//                 if(nGood<10)
//                     continue;

//                 for(int io =0; io<mCurrentFrame.N; io++)
//                     if(mCurrentFrame.mvbOutlier[io])
//                         mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

//                 // If few inliers, search by projection in a coarse window and optimize again
//                 if(nGood<50)
//                 {
//                     int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

//                     if(nadditional+nGood>=50)
//                     {
//                         nGood = Optimizer::PoseOptimization(&mCurrentFrame);

//                         // If many inliers but still not enough, search by projection again in a narrower window
//                         // the camera has been already optimized with many points
//                         if(nGood>30 && nGood<50)
//                         {
//                             sFound.clear();
//                             for(int ip =0; ip<mCurrentFrame.N; ip++)
//                                 if(mCurrentFrame.mvpMapPoints[ip])
//                                     sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
//                             nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

//                             // Final optimization
//                             if(nGood+nadditional>=50)
//                             {
//                                 nGood = Optimizer::PoseOptimization(&mCurrentFrame);

//                                 for(int io =0; io<mCurrentFrame.N; io++)
//                                     if(mCurrentFrame.mvbOutlier[io])
//                                         mCurrentFrame.mvpMapPoints[io]=NULL;
//                             }
//                         }
//                     }
//                 }


//                 // If the pose is supported by enough inliers stop ransacs and continue
//                 if(nGood>=50)
//                 {
//                     bMatch = true;
//                     break;
//                 }
//             }
//         }
//     }

//     if(!bMatch)
//     {
//         return false;
//     }
//     else
//     {
//         mnLastRelocFrameId = mCurrentFrame.mnId;
//         cout << "Relocalized!!" << endl;
//         return true;
//     }

// }

// void Tracking::Reset(bool bLocMap)
// {
//     Verbose::PrintMess("System Reseting", Verbose::VERBOSITY_NORMAL);

//     if(mpViewer)
//     {
//         mpViewer->RequestStop();
//         while(!mpViewer->isStopped())
//             usleep(3000);
//     }

//     // Reset Local Mapping
//     if (!bLocMap)
//     {
//         Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_NORMAL);
//         mpLocalMapper->RequestReset();
//         Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
//     }


//     // Reset Loop Closing
//     Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
//     mpLoopClosing->RequestReset();
//     Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

//     // Clear BoW Database
//     Verbose::PrintMess("Reseting Database...", Verbose::VERBOSITY_NORMAL);
//     mpKeyFrameDB->clear();
//     Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

//     // Clear Map (this erase MapPoints and KeyFrames)
//     mpAtlas->clearAtlas();
//     mpAtlas->CreateNewMap();
//     if (mSensor==System::IMU_STEREO || mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_RGBD)
//         mpAtlas->SetInertialSensor();
//     mnInitialFrameId = 0;

//     KeyFrame::nNextId = 0;
//     Frame::nNextId = 0;
//     mState = NO_IMAGES_YET;

//     mbReadyToInitializate = false;
//     mbSetInit=false;

//     mlRelativeFramePoses.clear();
//     mlpReferences.clear();
//     mlFrameTimes.clear();
//     mlbLost.clear();
//     mCurrentFrame = Frame();
//     mnLastRelocFrameId = 0;
//     mLastFrame = Frame();
//     mpReferenceKF = static_cast<KeyFrame*>(NULL);
//     mpLastKeyFrame = static_cast<KeyFrame*>(NULL);
//     mvIniMatches.clear();

//     if(mpViewer)
//         mpViewer->Release();

//     Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
// }

// void Tracking::ResetActiveMap(bool bLocMap)
// {
//     Verbose::PrintMess("Active map Reseting", Verbose::VERBOSITY_NORMAL);
//     if(mpViewer)
//     {
//         mpViewer->RequestStop();
//         while(!mpViewer->isStopped())
//             usleep(3000);
//     }

//     Map* pMap = mpAtlas->GetCurrentMap();

//     if (!bLocMap)
//     {
//         Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_VERY_VERBOSE);
//         mpLocalMapper->RequestResetActiveMap(pMap);
//         Verbose::PrintMess("done", Verbose::VERBOSITY_VERY_VERBOSE);
//     }

//     // Reset Loop Closing
//     Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
//     mpLoopClosing->RequestResetActiveMap(pMap);
//     Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

//     // Clear BoW Database
//     Verbose::PrintMess("Reseting Database", Verbose::VERBOSITY_NORMAL);
//     mpKeyFrameDB->clearMap(pMap); // Only clear the active map references
//     Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

//     // Clear Map (this erase MapPoints and KeyFrames)
//     mpAtlas->clearMap();


//     //KeyFrame::nNextId = mpAtlas->GetLastInitKFid();
//     //Frame::nNextId = mnLastInitFrameId;
//     mnLastInitFrameId = Frame::nNextId;
//     //mnLastRelocFrameId = mnLastInitFrameId;
//     mState = NO_IMAGES_YET; //NOT_INITIALIZED;

//     mbReadyToInitializate = false;

//     list<bool> lbLost;
//     // lbLost.reserve(mlbLost.size());
//     unsigned int index = mnFirstFrameId;
//     cout << "mnFirstFrameId = " << mnFirstFrameId << endl;
//     for(Map* pMap : mpAtlas->GetAllMaps())
//     {
//         if(pMap->GetAllKeyFrames().size() > 0)
//         {
//             if(index > pMap->GetLowerKFID())
//                 index = pMap->GetLowerKFID();
//         }
//     }

//     //cout << "First Frame id: " << index << endl;
//     int num_lost = 0;
//     cout << "mnInitialFrameId = " << mnInitialFrameId << endl;

//     for(list<bool>::iterator ilbL = mlbLost.begin(); ilbL != mlbLost.end(); ilbL++)
//     {
//         if(index < mnInitialFrameId)
//             lbLost.push_back(*ilbL);
//         else
//         {
//             lbLost.push_back(true);
//             num_lost += 1;
//         }

//         index++;
//     }
//     cout << num_lost << " Frames set to lost" << endl;

//     mlbLost = lbLost;

//     mnInitialFrameId = mCurrentFrame.mnId;
//     mnLastRelocFrameId = mCurrentFrame.mnId;

//     mCurrentFrame = Frame();
//     mLastFrame = Frame();
//     mpReferenceKF = static_cast<KeyFrame*>(NULL);
//     mpLastKeyFrame = static_cast<KeyFrame*>(NULL);
//     mvIniMatches.clear();

//     mbVelocity = false;

//     if(mpViewer)
//         mpViewer->Release();

//     Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
// }

// vector<MapPoint*> Tracking::GetLocalMapMPS()
// {
//     return mvpLocalMapPoints;
// }

// void Tracking::ChangeCalibration(const string &strSettingPath)
// {
//     cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
//     float fx = fSettings["Camera.fx"];
//     float fy = fSettings["Camera.fy"];
//     float cx = fSettings["Camera.cx"];
//     float cy = fSettings["Camera.cy"];

//     mK_.setIdentity();
//     mK_(0,0) = fx;
//     mK_(1,1) = fy;
//     mK_(0,2) = cx;
//     mK_(1,2) = cy;

//     cv::Mat K = cv::Mat::eye(3,3,CV_32F);
//     K.at<float>(0,0) = fx;
//     K.at<float>(1,1) = fy;
//     K.at<float>(0,2) = cx;
//     K.at<float>(1,2) = cy;
//     K.copyTo(mK);

//     cv::Mat DistCoef(4,1,CV_32F);
//     DistCoef.at<float>(0) = fSettings["Camera.k1"];
//     DistCoef.at<float>(1) = fSettings["Camera.k2"];
//     DistCoef.at<float>(2) = fSettings["Camera.p1"];
//     DistCoef.at<float>(3) = fSettings["Camera.p2"];
//     const float k3 = fSettings["Camera.k3"];
//     if(k3!=0)
//     {
//         DistCoef.resize(5);
//         DistCoef.at<float>(4) = k3;
//     }
//     DistCoef.copyTo(mDistCoef);

//     mbf = fSettings["Camera.bf"];

//     Frame::mbInitialComputations = true;
// }

// void Tracking::InformOnlyTracking(const bool &flag)
// {
//     mbOnlyTracking = flag;
// }

// void Tracking::UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame* pCurrentKeyFrame)
// {
//     Map * pMap = pCurrentKeyFrame->GetMap();
//     unsigned int index = mnFirstFrameId;
//     list<ORB_SLAM3::KeyFrame*>::iterator lRit = mlpReferences.begin();
//     list<bool>::iterator lbL = mlbLost.begin();
//     for(auto lit=mlRelativeFramePoses.begin(),lend=mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lbL++)
//     {
//         if(*lbL)
//             continue;

//         KeyFrame* pKF = *lRit;

//         while(pKF->isBad())
//         {
//             pKF = pKF->GetParent();
//         }

//         if(pKF->GetMap() == pMap)
//         {
//             (*lit).translation() *= s;
//         }
//     }

//     mLastBias = b;

//     mpLastKeyFrame = pCurrentKeyFrame;

//     mLastFrame.SetNewBias(mLastBias);
//     mCurrentFrame.SetNewBias(mLastBias);

//     while(!mCurrentFrame.imuIsPreintegrated())
//     {
//         usleep(500);
//     }


//     if(mLastFrame.mnId == mLastFrame.mpLastKeyFrame->mnFrameId)
//     {
//         mLastFrame.SetImuPoseVelocity(mLastFrame.mpLastKeyFrame->GetImuRotation(),
//                                       mLastFrame.mpLastKeyFrame->GetImuPosition(),
//                                       mLastFrame.mpLastKeyFrame->GetVelocity());
//     }
//     else
//     {
//         const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
//         const Eigen::Vector3f twb1 = mLastFrame.mpLastKeyFrame->GetImuPosition();
//         const Eigen::Matrix3f Rwb1 = mLastFrame.mpLastKeyFrame->GetImuRotation();
//         const Eigen::Vector3f Vwb1 = mLastFrame.mpLastKeyFrame->GetVelocity();
//         float t12 = mLastFrame.mpImuPreintegrated->dT;

//         mLastFrame.SetImuPoseVelocity(IMU::NormalizeRotation(Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
//                                       twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
//                                       Vwb1 + Gz*t12 + Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
//     }

//     if (mCurrentFrame.mpImuPreintegrated)
//     {
//         const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);

//         const Eigen::Vector3f twb1 = mCurrentFrame.mpLastKeyFrame->GetImuPosition();
//         const Eigen::Matrix3f Rwb1 = mCurrentFrame.mpLastKeyFrame->GetImuRotation();
//         const Eigen::Vector3f Vwb1 = mCurrentFrame.mpLastKeyFrame->GetVelocity();
//         float t12 = mCurrentFrame.mpImuPreintegrated->dT;

//         mCurrentFrame.SetImuPoseVelocity(IMU::NormalizeRotation(Rwb1*mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
//                                       twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
//                                       Vwb1 + Gz*t12 + Rwb1*mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
//     }

//     mnFirstImuFrameId = mCurrentFrame.mnId;
// }

// void Tracking::NewDataset()
// {
//     mnNumDataset++;
// }

// int Tracking::GetNumberDataset()
// {
//     return mnNumDataset;
// }

// int Tracking::GetMatchesInliers()
// {
//     return mnMatchesInliers;
// }

// void Tracking::SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, string strFolder)
// {
//     mpSystem->SaveTrajectoryEuRoC(strFolder + strNameFile_frames);
//     //mpSystem->SaveKeyFrameTrajectoryEuRoC(strFolder + strNameFile_kf);
// }

// void Tracking::SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, Map* pMap)
// {
//     mpSystem->SaveTrajectoryEuRoC(strNameFile_frames, pMap);
//     if(!strNameFile_kf.empty())
//         mpSystem->SaveKeyFrameTrajectoryEuRoC(strNameFile_kf, pMap);
// }

// float Tracking::GetImageScale()
// {
//     return mImageScale;
// }

// #ifdef REGISTER_LOOP
// void Tracking::RequestStop()
// {
//     unique_lock<mutex> lock(mMutexStop);
//     mbStopRequested = true;
// }

// bool Tracking::Stop()
// {
//     unique_lock<mutex> lock(mMutexStop);
//     if(mbStopRequested && !mbNotStop)
//     {
//         mbStopped = true;
//         cout << "Tracking STOP" << endl;
//         return true;
//     }

//     return false;
// }

// bool Tracking::stopRequested()
// {
//     unique_lock<mutex> lock(mMutexStop);
//     return mbStopRequested;
// }

// bool Tracking::isStopped()
// {
//     unique_lock<mutex> lock(mMutexStop);
//     return mbStopped;
// }

// void Tracking::Release()
// {
//     unique_lock<mutex> lock(mMutexStop);
//     mbStopped = false;
//     mbStopRequested = false;
// }
// #endif

} //namespace ORB_SLAM
