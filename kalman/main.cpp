#include <iostream>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/videoio.hpp"

// 扣的代码
#include <ShortPathCalculator.h>
#include <VideoExample.h>
#include <defines.h>

using namespace cv;
using namespace std;

// tracker settings
std::unique_ptr<BaseTracker> m_tracker;  //这个不得不放在最前边了
TrackerSettings m_trackerSettings;
float m_fps = 25;

void drawText(Mat& image);
CascadeClassifier faceCascade;
vector<Point> DetectFace(Mat, Mat);
void Detection(FrameInfo& frame);
bool InitTracker();
void Tracking(FrameInfo& frame);

int main() {
  cout << "Built with OpenCV " << CV_VERSION << endl;
  Mat image, imgGray;
  VideoCapture capture;
  // load model
  bool load_model = faceCascade.load("haarcascade_frontalface_alt2.xml");
  if (load_model) {
    cout << "load model success!" << endl;
  } else {
    cout << "load model failed." << endl;
  }

  //   // 1.kalman filter setup
  //   const int stateNum = 4;
  //   const int measureNum = 2;
  //   // h[4x2] * m[2x1] = s[4x1]
  //   KalmanFilter KF(stateNum, measureNum, 0);  // 状态 x y dx dy
  //                                              // 观测 x y
  //                                              // 控制 无
  //   Mat state(stateNum, 1, CV_32FC1);          // state(x,y,detaX,detaY)
  //   Mat processNoise(stateNum, 1, CV_32F);
  //   Mat measurement = Mat::zeros(measureNum, 1, CV_32F);  // measurement(x,y)
  //   cout << "initial measurement :\n" << measurement << endl;
  //   randn(state, Scalar::all(0), Scalar::all(0.1));  //随机生成一个矩阵，期望是0，标准差为0.1;
  //   cout << "random initial state :\n" << state << endl;

  //   // 转移矩阵
  //   KF.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);  //元素导入矩阵，按行;
  //   //   cout << "transition matrix :\n" << KF.transitionMatrix << endl;
  //   //!< measurement matrix (H) 观测模型
  //   setIdentity(KF.measurementMatrix);
  //   // KF.measurementMatrix = (Mat_<float>(4, 2) << 1,0,0,0,0,1,0,0);//元素导入矩阵，按行;
  //   //   cout << "measurementMatrix matrix :\n" << KF.measurementMatrix << endl;

  //   // wk 是过程噪声，并假定其符合均值为零，协方差矩阵为Qk(Q)的多元正态分布;
  //   setIdentity(KF.processNoiseCov, Scalar::all(1e-5));
  //   //   cout << "processNoiseCov matrix :\n" << KF.processNoiseCov << endl;

  //   // vk 是观测噪声，其均值为零，协方差矩阵为Rk,且服从正态分布;
  //   setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
  //   //   cout << "measurementNoiseCov matrix :\n" << KF.measurementNoiseCov << endl;

  //   //!< priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)*/  A代表F: transitionMatrix
  //   //预测估计协方差矩阵;
  //   setIdentity(KF.errorCovPost, Scalar::all(1));
  //   //   cout << "errorCovPost matrix :\n" << KF.errorCovPost << endl;

  //   // initialize post state of kalman filter at random
  //   // randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));
  //   KF.statePost = (Mat_<float>(4, 1) << 100., 100., 0., 0.);  //元素导入矩阵，按行;
  //   cout << "statePost matrix :\n" << KF.statePost << endl;

  // 初始化frame
  // 这变量在Loop外
  // 我们这个frame里还是要存图片的，temi里应该连图片都不用存了
  FrameInfo frameInfo;
  int framesCounter = 1;                // frame计数器
  bool m_isTrackerInitialized = false;  // 初始化tracker
  capture.open(0);

  if (capture.isOpened()) {
    cout << "Capture is opened" << endl;
  } else {
    cout << "No capture" << endl;
  }

  for (;;) {

    // 读取数据到FrameInfo里  
    capture >> frameInfo.m_frame.GetMatBGRWrite();  // 写入到帧里
    if (frameInfo.m_frame.empty()) break;           //如果没写进去就停
    frameInfo.m_frameInd = framesCounter;
    ++framesCounter;

    // 初始化tracker
    if (!m_isTrackerInitialized) {
      // 用第一帧初始化tracker
      m_isTrackerInitialized = InitTracker();  // 这i主要是根据setting来生成m_tracker
      if (!m_isTrackerInitialized) {
        std::cerr << "CaptureAndDetect: Tracker initialize error!!!" << std::endl;
        break;
      }
    }

    // new detection
    Detection(frameInfo);
    cv::Mat img_for_show = frameInfo.m_frame.GetMatBGR();
    for (int i = 0; i < frameInfo.m_centers.size(); i++) {
      circle(img_for_show, frameInfo.m_centers[i], 15, Scalar(255, 0, 0), 8);
    }
    imshow("img_for_show", img_for_show);
    waitKey(1);

    // new tracking
    Tracking(frameInfo);

    cv::Mat track_for_show = frameInfo.m_frame.GetMatBGR();
    for (int i = 0; i < frameInfo.m_track.size(); i++) {
      Point_t center_for_show;
      center_for_show = frameInfo.m_track[i].m_center;
      circle(track_for_show, center_for_show, 25, Scalar(0, 255, 0), 8);
      putText(track_for_show, frameInfo.m_track[i].m_ID.ID2Str(), center_for_show, FONT_HERSHEY_PLAIN, 2.0, Scalar(0, 0,255));
    }
    imshow("track_for_show", track_for_show);
    waitKey(1);
  }
  // }
  return 0;
}

// vector<Point> DetectFace(Mat img, Mat imgGray) {
//   namedWindow("src", WINDOW_AUTOSIZE);
//   vector<Rect> faces;
//   vector<Point> corner;
//   faceCascade.detectMultiScale(imgGray, faces, 1.2, 5, 0, Size(30, 30));
//   //   for (auto b : faces) {
//   //     cout << "输出一张人脸位置：(x,y):"
//   //          << "(" << b.x << "," << b.y << ") , (width,height):(" << b.width << "," << b.height << ")" << endl;
//   //   }
//   if (faces.size() > 0) {
//     for (size_t i = 0; i < faces.size(); i++) {
//       putText(img, "handsome man!", Point(faces[i].x, faces[i].y - 10), FONT_HERSHEY_PLAIN, 2.0, Scalar(0, 0, 255));
//       rectangle(img, Point(faces[i].x, faces[i].y), Point(faces[i].x + faces[i].width, faces[i].y + faces[i].height), Scalar(0, 0, 255), 1, 8);
//       cout << faces[i] << endl;
//       //
//       corner.push_back(Point(faces[i].x, faces[i].y));
//       circle(img, Point(faces[i].x, faces[i].y), 15, Scalar(255, 0, 0), 8);
//     }
//     ////////////////////////////
//     // 把rect 放到regions里
//     regions_t m_regions;
//     for (auto rect : faces) {
//       m_regions.push_back(rect);
//     }
//     for (auto reg : m_regions) {
//       cout << reg.m_brect << endl;
//     }
//     ///////////////////////////
//   }
//   return corner;
// }

/**
 * @brief 给FrameInfo的center写入数据
 *
 */
void Detection(FrameInfo& frame) {
  cv::UMat frame_u;  // 传进来的frame里可能有多帧,这个是根据batch确定的
  frame_u = frame.m_frame.GetUMatGray();
  frame.CleanCenters();  // 清空

  vector<Rect> faces;
  faceCascade.detectMultiScale(frame_u, faces, 1.2, 5, 0, Size(30, 30));
  for (auto rect : faces) {
    // 把检测框的tl加入到frameInfo里，之后就track这个
    Point_t tmp_center;
    tmp_center.x = rect.tl().x;
    tmp_center.y = rect.tl().y;
    // std::cout << "center x :" << tmp_center.x << std::endl;
    // std::cout << "center y :" << tmp_center.y << std::endl;
    frame.m_centers.push_back(tmp_center);
  }
//   std::cout << "m_centers size: " << frame.m_centers.size() << std::endl;
}

// 根据配置文件生成一个CTracker
bool InitTracker() {
  //   if (!m_trackerSettingsLoaded) {
  m_trackerSettings.SetDistance(tracking::DistCenters);
  m_trackerSettings.m_kalmanType = tracking::KalmanLinear;
  m_trackerSettings.m_filterGoal = tracking::FilterCenter;
  m_trackerSettings.m_lostTrackType =
      tracking::TrackKCF;  // Use visual objects tracker for collisions resolving. Used if m_filterGoal == tracking::FilterRect
  m_trackerSettings.m_matchType = tracking::MatchHungrian;
  m_trackerSettings.m_dt = 0.3f;             // Delta time for Kalman filter
  m_trackerSettings.m_accelNoiseMag = 0.1f;  // Accel noise magnitude for Kalman filter
  m_trackerSettings.m_distThres = 0.8f;      // Distance threshold between region and object on two frames
  m_trackerSettings.m_minAreaRadiusPix = 20.f; 
  m_trackerSettings.m_maximumAllowedSkippedFrames = cvRound(m_fps / 2);  // Maximum allowed skipped frames
  m_trackerSettings.m_maxTraceLength = cvRound(5 * m_fps);               // Maximum trace length
                                                                         //   }
  // 这里是创建一个CTracker
  m_tracker = BaseTracker::CreateTracker(m_trackerSettings);
  cout << "Created a tracker" << endl;
  return true;
}

/**
 * @brief 通过CTracker对检测出的物体进行track
 */
void Tracking(FrameInfo& frame) {
  frame.CleanTracks();  // 每个loop都要clean一下
                        // 遍历每一帧
    // 把center传进去
    m_tracker->Update(frame.m_centers, frame.m_frame.GetUMatGray(), m_fps);
    m_tracker->GetTracks(frame.m_track);
}