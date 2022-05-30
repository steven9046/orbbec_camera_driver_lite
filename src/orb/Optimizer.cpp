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

#include "Optimizer.h"

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <complex>
#include <unsupported/Eigen/MatrixFunctions>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/sparse_block_matrix.h"
#include "g2o/solvers/linear_solver_dense.h"
#include "g2o/solvers/linear_solver_eigen.h"
#include "g2o/types/types_six_dof_expmap.h"
// #include "G2oTypes.h" // 我们这里好像没用到
#include <mutex>

#include "CameraModels/Converter.h"
#include "OptimizableTypes.h"

namespace ORB_SLAM3 {

/**
 * @brief 对位姿进行优化求解
 * 1. 构建优化求解器
 * 2. 设置顶点,这里是一元边，顶点只有一个，就是相机的位姿
 * 3. 设置边，这里用地图点的重投影作为一元边来约束顶点，分为单目观测地图点和双目观测地图点
 * 		误差项为初始特征提取得到的像素坐标(x1,y1)和重投影得到的像素坐标(x2,y2)的差值
 * 4. 执行优化
 */
int PoseOptimization(Frame* pFrame) {
  // step1: 优化求解器
  g2o::SparseOptimizer optimizer;
  // 设置优化求解器: 线性稠密，LM
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver;
  linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
  g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);


  int nInitialCorrespondences = 0; // 初始匹配数量
  
  // step2: 设置顶点
  // 设立应该是se3->SE3的映射Expmap
  g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();  //
  Sophus::SE3<float> Tcw = pFrame->GetPose();               // 得到变换矩阵
//   Sophus::SE3<float> Tcw = Sophus::SE3f();
//   printf("Getting Pose\n");
//   float* pos1 = Tcw.data();
//   for (int i = 0; i < 7; i++) {
//     printf("%f ", pos1[i]);
//   }
//   printf("\n");
  // 这里SE3Quat应该是因为se3的旋转和so3的映射不一样,so3里应该只需要设置一个四元数
  // se3里的quat是一个四元数加一个平移向量
  // 这里setEstimate函数只是把数据传进去
  vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>()));
  vSE3->setId(0);
  vSE3->setFixed(false);
  optimizer.addVertex(vSE3);

  // step2: 设置边
  const int N = pFrame->N;
  // 单目观测地图点边
  vector<ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
  vector<size_t> vnIndexEdgeMono;
  vpEdgesMono.reserve(N);
  vnIndexEdgeMono.reserve(N);
  // 双目观测地图点边
  vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
  vector<size_t> vnIndexEdgeStereo;
  vpEdgesStereo.reserve(N);
  vnIndexEdgeStereo.reserve(N);

  const float deltaMono = sqrt(5.991);
  const float deltaStereo = sqrt(7.815);

  {
    // unique_lock<mutex> lock(MapPoint::mGlobalMutex);
    for (int i = 0; i < N; i++) {
      MapPoint* pMP = pFrame->mvpMapPoints[i];
      if (pMP) {
        // Monocular observation
        if (pFrame->mvuRight[i] < 0) {
		  // 计数	
          nInitialCorrespondences++;
		  // 记录该地图点是不是外点，以此判断是否后续使用
          pFrame->mvbOutlier[i] = false;

		  // 设置边的观测值
          Eigen::Matrix<double, 2, 1> obs;
          const cv::KeyPoint& kpUn = pFrame->mvKeysUn[i];
          obs << kpUn.pt.x, kpUn.pt.y;
		  
		  // 单目边
          ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose* e = new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose();
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));// 只连接一个顶点
          e->setMeasurement(obs);// 设置通过特征匹配得到的重投影坐标
          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);// 信息矩阵，不同尺度不一样
          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);// 鲁棒核
          rk->setDelta(deltaMono);
		  // 设置相机参数及3D坐标，用来计算重投影
          e->pCamera = pFrame->mpCamera;
          e->Xw = pMP->GetWorldPos().cast<double>();
		  // 添加边
          optimizer.addEdge(e);
		  // 计数
          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        } else {
          nInitialCorrespondences++;
          pFrame->mvbOutlier[i] = false;
		  
		  // 双目观测比单目多一个虚拟右目上的x坐标
          Eigen::Matrix<double, 3, 1> obs;
          const cv::KeyPoint& kpUn = pFrame->mvKeysUn[i];
          const float& kp_ur = pFrame->mvuRight[i];
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
          e->setMeasurement(obs);
          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
          Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
          e->setInformation(Info);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(deltaStereo);

          e->fx = pFrame->fx;
          e->fy = pFrame->fy;
          e->cx = pFrame->cx;
          e->cy = pFrame->cy;
          e->bf = pFrame->mbf;
          e->Xw = pMP->GetWorldPos().cast<double>();

          optimizer.addEdge(e);

          vpEdgesStereo.push_back(e);
          vnIndexEdgeStereo.push_back(i);
        }
      }
    }
  }
//   printf("nInitialCorrespondences: %d\n", nInitialCorrespondences);
  if (nInitialCorrespondences < 3) return 0;

  // step4:执行优化
  // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
  // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
  const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
  const int its[4] = {10, 10, 10, 10};

  int nBad = 0;
  for (size_t it = 0; it < 4; it++) {
    // 每一轮需要重新设置一下优化初始值
    Tcw = pFrame->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>()));
    //进行优化
    optimizer.initializeOptimization(0);
    optimizer.optimize(10);
    //遍历所有边，重新计算误差，以此来划分内点/外点
    int nBad = 0;
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

      const size_t idx = vnIndexEdgeMono[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      // 计算出的误差
      const float chi2 = e->chi2();

      // 如果超过阈值，鉴定为坏，否则还可以标记为内点
      if (chi2 > chi2Mono[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBad++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
      }

      if (it == 2) e->setRobustKernel(0);
    }
    if (optimizer.edges().size() < 10) break;
  }
  // 重新设置Frame为优化后的位姿
  // Recover optimized pose and return number of inliers
  g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
  g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
  Sophus::SE3<float> pose(SE3quat_recov.rotation().cast<float>(), SE3quat_recov.translation().cast<float>());
  pFrame->SetPose(pose);
  printf("after opt: \n");
  float* pos2 = pose.data();
  for (int i = 0; i < 7; i++) {
    printf("%f ", pos2[i]);
  }
  printf("\n");
  return nInitialCorrespondences - nBad;
}

}  // namespace ORB_SLAM3
