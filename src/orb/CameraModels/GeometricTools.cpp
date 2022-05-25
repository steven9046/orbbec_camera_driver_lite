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

#include "GeometricTools.h"

// #include "KeyFrame.h"

namespace ORB_SLAM3
{

// Eigen::Matrix3f GeometricTools::ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2)
// {
//     Sophus::SE3<float> Tc1w = pKF1->GetPose();
//     Sophus::Matrix3<float> Rc1w = Tc1w.rotationMatrix();
//     Sophus::SE3<float>::TranslationMember tc1w = Tc1w.translation();

//     Sophus::SE3<float> Tc2w = pKF2->GetPose();
//     Sophus::Matrix3<float> Rc2w = Tc2w.rotationMatrix();
//     Sophus::SE3<float>::TranslationMember tc2w = Tc2w.translation();

//     Sophus::Matrix3<float> Rc1c2 = Rc1w * Rc2w.transpose();
//     Eigen::Vector3f tc1c2 = -Rc1c2 * tc2w + tc1w;

//     Eigen::Matrix3f tc1c2x = Sophus::SO3f::hat(tc1c2);

//     const Eigen::Matrix3f K1 = pKF1->mpCamera->toK_();
//     const Eigen::Matrix3f K2 = pKF2->mpCamera->toK_();

//     return K1.transpose().inverse() * tc1c2x * Rc1c2 * K2.inverse();
// }

/**
 * @brief 
 * @param [in]  x_c1    Frame1 里对应的 地图点3D坐标
 * @param [in]  x_c2    Frame2 里对应的 地图点3D坐标
 * @param [in]  Tc1w    Frame1 的世界坐标
 * @param [in]  Tc2w    Frame2 的世界坐标
 * @param [out] x3D     输出地图点世界系3D坐标
 * x_c1 = Tc1w * x3D
 * x_c2 = Tc2w * x3D
 * https://zhuanlan.zhihu.com/p/51835837 类似
 */
bool GeometricTools::Triangulate(Eigen::Vector3f &x_c1, Eigen::Vector3f &x_c2,Eigen::Matrix<float,3,4> &Tc1w ,Eigen::Matrix<float,3,4> &Tc2w , Eigen::Vector3f &x3D)
{
    Eigen::Matrix4f A;
    // .block<提取矩阵size>(起始位置)
    // A的第一行 = x * [a31 a32 a33 a34] - [a11 a12 a13 a14]
    // x_c1: x,y,z
    // Tc1w:
    // a11 a12 a13 a14
    // a21 a22 a23 a24
    // a31 a32 a33 a34
    // a41 a42 a43 a44
    A.block<1,4>(0,0) = x_c1(0) * Tc1w.block<1,4>(2,0) - Tc1w.block<1,4>(0,0);
    A.block<1,4>(1,0) = x_c1(1) * Tc1w.block<1,4>(2,0) - Tc1w.block<1,4>(1,0);
    A.block<1,4>(2,0) = x_c2(0) * Tc2w.block<1,4>(2,0) - Tc2w.block<1,4>(0,0);
    A.block<1,4>(3,0) = x_c2(1) * Tc2w.block<1,4>(2,0) - Tc2w.block<1,4>(1,0);

    Eigen::JacobiSVD<Eigen::Matrix4f> svd(A, Eigen::ComputeFullV);

    Eigen::Vector4f x3Dh = svd.matrixV().col(3);

    if(x3Dh(3)==0)
        return false;

    // Euclidean coordinates
    x3D = x3Dh.head(3)/x3Dh(3);

    return true;
}

} //namespace ORB_SLAM
