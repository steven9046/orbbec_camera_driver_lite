#include "kalman/track.h"

/**
 * @brief CTrack 主要是完成卡尔曼的工作， 有一个目标就会生成一个CTrack，通过assignment匹配好的物体来作为该目标的观测
 * @param pt
 * @param region
 * @param deltaTime
 * @param accelNoiseMag
 * @param trackID
 * @param filterObjectSize
 * @param externalTrackerForLost
 */
CTrack::CTrack(const Point_t& center,
               tracking::KalmanType kalmanType,
               track_t deltaTime,
               track_t accelNoiseMag,
               bool useAcceleration,
               track_id_t trackID,
               bool filterObjectSize,
               tracking::LostTrackType externalTrackerForLost)
    :
      m_kalman(deltaTime, accelNoiseMag),
      m_predictionPoint(center),
      m_trackID(trackID),
      m_externalTrackerForLost(externalTrackerForLost)
{
    m_kalman.Update(m_predictionPoint, true);
    Point_t pt(center.x, center.y);
    m_trace.push_back(pt, pt);
}

/**
 * @brief 计算两个点的距离
 */
track_t CTrack::CalcDistCenter(const CRegion& reg) const
{
    Point_t diff = m_predictionPoint - reg.m_rrect.center;
    return sqrtf(sqr(diff.x) + sqr(diff.y));
}

///
/// \brief CTrack::Update
/// \param region
/// \param dataCorrect
/// \param max_trace_length
/// \param prevFrame
/// \param currFrame
/// \param trajLen
///
void CTrack::Update(const Point_t& center,
                    bool dataCorrect, // 有assignment时为true,没有时为false
                    size_t max_trace_length,// setting 里的值 50
                    cv::UMat prevFrame, // 前一帧
                    cv::UMat currFrame, // 当前帧
                    int trajLen, 
                    int maxSpeedForStatic)// setting 里设置 10
{
    // 我们只追踪点,m_predictionPoint得到了更新
    PointUpdate(center, dataCorrect);

    if (dataCorrect)
    {
        m_lastCenter = center; // 保存上一个
        m_trace.push_back(m_predictionPoint, center); // 轨迹保存起来
        // CheckStatic(trajLen, currFrame, region, maxSpeedForStatic);
    }
    else
    {
        m_trace.push_back(m_predictionPoint);
    }
    // 处理溢出
    if (m_trace.size() > max_trace_length)
        m_trace.pop_front(m_trace.size() - max_trace_length);
}

///
/// \brief CTrack::IsStatic
/// \return
///
bool CTrack::IsStatic() const
{
    return m_isStatic;
}

///
/// \brief CTrack::IsStaticTimeout
/// \param framesTime
/// \return
///
bool CTrack::IsStaticTimeout(int framesTime) const
{
    return (m_staticFrames > framesTime);
}

///
/// \brief CTrack::IsOutOfTheFrame
/// \return
///
bool CTrack::IsOutOfTheFrame() const
{
	return m_outOfTheFrame;
}

/**
 * @brief 根据卡尔曼预测的速度、预测的点、设置的minRadius算出point将要出现的位置
 */
cv::RotatedRect CTrack::CalcPredictionEllipse(cv::Size_<track_t> minRadius) const
{
	// Move ellipse to velocity
	auto velocity = m_kalman.GetVelocity();
	Point_t d(3.f * velocity[0], 3.f * velocity[1]);
	
	cv::RotatedRect rrect(m_predictionPoint, cv::Size2f(std::max<float>(minRadius.width, fabs(d.x)), std::max<float>(minRadius.height, fabs(d.y))), 0);

	if (fabs(d.x) + fabs(d.y) > 4) // pix
	{
		if (fabs(d.x) > 0.0001f)
		{
			track_t l = std::min(rrect.size.width, rrect.size.height) / 3;

			track_t p2_l = sqrtf(sqr(d.x) + sqr(d.y));
			rrect.center.x = l * d.x / p2_l + m_predictionPoint.x;
			rrect.center.y = l * d.y / p2_l + m_predictionPoint.y;

			rrect.angle = atanf(d.y / d.x);
		}
		else
		{
			rrect.center.y += d.y / 3;
			rrect.angle = static_cast<float>(CV_PI / 2.);
		}
	}
	return rrect;
}

///
/// \brief CTrack::IsInsideArea
///        If result <= 1 then center of the object is inside ellipse with prediction and velocity
/// \param pt
/// \return
///
track_t CTrack::IsInsideArea(const Point_t& pt, const cv::RotatedRect& rrect) const
{
	Point_t pt_(pt.x - rrect.center.x, pt.y - rrect.center.y);
	track_t r = sqrtf(sqr(pt_.x) + sqr(pt_.y));
	track_t t = (r > 1) ? acosf(pt_.x / r) : 0;
	track_t t_ = t - rrect.angle;
	Point_t pt_rotated(r * cosf(t_), r * sinf(t_));

	return sqr(pt_rotated.x) / sqr(rrect.size.width) + sqr(pt_rotated.y) / sqr(rrect.size.height);
}

// ///
// /// \brief CTrack::CheckStatic
// /// \param trajLen
// /// \return
// ///
// bool CTrack::CheckStatic(int trajLen, cv::UMat currFrame, const CRegion& region, int maxSpeedForStatic)
// {
//     if (!trajLen || static_cast<int>(m_trace.size()) < trajLen)
//     {
//         m_isStatic = false;
//         m_staticFrames = 0;
//         m_staticFrame = cv::UMat();
//     }
//     else
//     {
//         auto velocity = m_kalman.GetVelocity();
//         track_t speed = sqrt(sqr(velocity[0]) + sqr(velocity[1]));
//         if (speed < maxSpeedForStatic)
//         {
//             if (!m_isStatic)
//             {
//                 m_staticFrame = currFrame.clone();
//                 m_staticRect = region.m_brect;
//             }

//             ++m_staticFrames;
//             m_isStatic = true;
//         }
//         else
//         {
//             m_isStatic = false;
//             m_staticFrames = 0;
//             m_staticFrame = cv::UMat();
//         }
//     }
//     return m_isStatic;
// }

///
/// \brief CTrack::ConstructObject
/// \return
///
TrackingObject CTrack::ConstructObject() const
{
    return TrackingObject(GetLastCenter(), m_trackID, m_trace, IsStatic(), m_staticFrames, IsOutOfTheFrame(), m_kalman.GetVelocity());
}

/**
 * @brief 返回center，用于构建object
 */
Point_t CTrack::GetLastCenter() const
{
    return m_predictionPoint;
}

///
/// \brief CTrack::GetID
/// \return
///
track_id_t CTrack::GetID() const
{
    return m_trackID;
}

///
/// \brief CTrack::SkippedFrames
/// \return
///
size_t CTrack::SkippedFrames() const
{
    return m_skippedFrames;
}

///
/// \brief CTrack::SkippedFrames
/// \return
///
size_t& CTrack::SkippedFrames()
{
    return m_skippedFrames;
}

///
/// \param pt
/// \param dataCorrect
///
void CTrack::PointUpdate(const Point_t& pt,
                         bool dataCorrect)
{
    // 预测
    m_kalman.GetPointPrediction();
    // 更新
    m_predictionPoint = m_kalman.Update(pt, dataCorrect);
}
