#include "kalman/Kalman.h"
#include <iostream>
#include <vector>

/**
 * @brief Construct a new TKalmanFilter::TKalmanFilter object
 * @param deltaTime         事件变化，这个应该根据帧率来，也可以通过计算传入真实的时间流逝
 * @param accelNoiseMag     噪声增益，这个就是作用在初始的协方差Q
 */
TKalmanFilter::TKalmanFilter(track_t deltaTime,  // time increment (lower values makes target more "massive")
                             track_t accelNoiseMag)
    : m_accelNoiseMag(accelNoiseMag),
      m_deltaTime(deltaTime),
      m_deltaTimeMin(deltaTime),
      m_deltaTimeMax(2 * deltaTime){
  m_deltaStep = (m_deltaTimeMax - m_deltaTimeMin) / m_deltaStepsCount;  // 20
}

//              4状态  2观测
// 这里就是kalman滤波的初始化过程
void TKalmanFilter::CreateLinear(Point_t xy0, Point_t xyv0) {
  // We don't know acceleration, so, assume it to process noise.
  // But we can guess, the range of acceleration values thich can be achieved by tracked object.
  // Process noise. (standard deviation of acceleration: m/s^2)
  // shows, woh much target can accelerate.

  // 4 state variables, 2 measurements
  m_linearKalman.init(4, 2, 0, El_t);
  // Transition cv::Matrix
  // 这里就是一个二维速度模型(x,y,vx,vy)
  m_linearKalman.transitionMatrix = (cv::Mat_<track_t>(4, 4) << 1, 0, m_deltaTime, 0, 0, 1, 0, m_deltaTime, 0, 0, 1, 0, 0, 0, 0, 1);

  // init...
  m_lastPointResult = xy0;
  m_linearKalman.statePre.at<track_t>(0) = xy0.x;   // x
  m_linearKalman.statePre.at<track_t>(1) = xy0.y;   // y
  m_linearKalman.statePre.at<track_t>(2) = xyv0.x;  // vx
  m_linearKalman.statePre.at<track_t>(3) = xyv0.y;  // vy

  m_linearKalman.statePost.at<track_t>(0) = xy0.x;
  m_linearKalman.statePost.at<track_t>(1) = xy0.y;
  m_linearKalman.statePost.at<track_t>(2) = xyv0.x;
  m_linearKalman.statePost.at<track_t>(3) = xyv0.y;

  cv::setIdentity(m_linearKalman.measurementMatrix);

  // 这个噪声矩阵不知道为什么这么设
  m_linearKalman.processNoiseCov = (cv::Mat_<track_t>(4, 4) << pow(m_deltaTime, 4.0) / 4.0, 0, pow(m_deltaTime, 3.0) / 2.0, 0, 0,
                                    pow(m_deltaTime, 4.0) / 4.0, 0, pow(m_deltaTime, 3.0) / 2.0, pow(m_deltaTime, 3.0) / 2.0, 0,
                                    pow(m_deltaTime, 2.0), 0, 0, pow(m_deltaTime, 3.0) / 2.0, 0, pow(m_deltaTime, 2.0));

  m_linearKalman.processNoiseCov *= m_accelNoiseMag;

  cv::setIdentity(m_linearKalman.measurementNoiseCov, cv::Scalar::all(0.1));

  cv::setIdentity(m_linearKalman.errorCovPost, cv::Scalar::all(.1));

  m_initialPoints.reserve(MIN_INIT_VALS);

  m_initialized = true;
}

//---------------------------------------------------------------------------
// 返回预测到的位置点,在程序里执行时需要先执行这个，然后在去更新
Point_t TKalmanFilter::GetPointPrediction() {
  if (m_initialized) {
    cv::Mat prediction;
    prediction = m_linearKalman.predict();
    m_lastPointResult = Point_t(prediction.at<track_t>(0), prediction.at<track_t>(1));
  }
  return m_lastPointResult;
}

//---------------------------------------------------------------------------
/**
 * @brief
 * @param pt          观测数据
 * @param dataCorrect 控制是否用观测来更新，如果是false就不用观测数据
 * @return            返回值是用观测更新过的点
 */
Point_t TKalmanFilter::Update(Point_t pt, bool dataCorrect) {
  // 如果没有初始化，这里不太可能进来，除非直接就Update,不create
  if (!m_initialized) {
    if (m_initialPoints.size() < MIN_INIT_VALS) {
      if (dataCorrect) {
        m_initialPoints.push_back(pt);
        m_lastPointResult = pt;
      }
    }
    if (m_initialPoints.size() == MIN_INIT_VALS) {
      track_t kx = 0;
      track_t bx = 0;
      track_t ky = 0;
      track_t by = 0;
      // 这里这个线性回归是什么意思
      get_lin_regress_params(m_initialPoints, 0, MIN_INIT_VALS, kx, bx, ky, by);
      Point_t xy0(kx * (MIN_INIT_VALS - 1) + bx, ky * (MIN_INIT_VALS - 1) + by);
      Point_t xyv0(kx, ky);
      // 这里type不用了，我们只有这一个type
      CreateLinear(xy0, xyv0);  // 初始化
      m_lastDist = 0;
    }
  }

  if (m_initialized) {
    cv::Mat measurement(2, 1, Mat_t(1));
    // 是否使用观测进行更新
    if (!dataCorrect) {
      measurement.at<track_t>(0) = m_lastPointResult.x;  // update using prediction
      measurement.at<track_t>(1) = m_lastPointResult.y;
    } else {
      measurement.at<track_t>(0) = pt.x;  // update using measurements
      measurement.at<track_t>(1) = pt.y;
    }
    // 使用测量进行更新
    cv::Mat estimated;
    estimated = m_linearKalman.correct(measurement);

    // Inertia correction 惯性修正，这里不使用加速度模型时要用这个惯性修正
    track_t currDist = sqrtf(sqr(estimated.at<track_t>(0) - pt.x) + sqr(estimated.at<track_t>(1) - pt.y));
    if (currDist > m_lastDist)
      m_deltaTime = std::min(m_deltaTime + m_deltaStep, m_deltaTimeMax);
    else
      m_deltaTime = std::max(m_deltaTime - m_deltaStep, m_deltaTimeMin);

    m_lastDist = currDist;
    m_linearKalman.transitionMatrix.at<track_t>(0, 2) = m_deltaTime;
    m_linearKalman.transitionMatrix.at<track_t>(1, 3) = m_deltaTime;

    // 记录上次结果
    m_lastPointResult.x = estimated.at<track_t>(0);  // update using measurements
    m_lastPointResult.y = estimated.at<track_t>(1);
  } else  // 这个不太可能，因为只要create了，就初始化好了
  {
    if (dataCorrect) m_lastPointResult = pt;
  }
  return m_lastPointResult;
}

//---------------------------------------------------------------------------
//  根据状态空间不同，返回相应的速度
cv::Vec<track_t, 2> TKalmanFilter::GetVelocity() const {
  cv::Vec<track_t, 2> res(0, 0);
  if (m_initialized) {
    int indX = 2;
    int indY = 3;
    res[0] = m_linearKalman.statePre.at<track_t>(indX);
    res[1] = m_linearKalman.statePre.at<track_t>(indY);
  }
  return res;
}
