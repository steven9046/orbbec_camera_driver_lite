#include "kalman/Ctracker.h"
#include "kalman/ShortPathCalculator.h"
#include "kalman/track.h"

/**
 * @brief CTracker就是进行track的主体，把frame的数据传进来，就可以track,他的主要功能就是创建，更新，删除ctrack
 *        // 这里需要更改的，CTrack用frame里的center，不用region
 *        在最外层就执行了一个实例化，之后就是loop里的update
 *        1. 构造函数会创建一个匈牙利算法(分配器)
 *        2. 用frame的数据更新
 */
class CTracker final : public BaseTracker  // final 关键字，禁止继承
{
 public:
  CTracker(const TrackerSettings& settings);
  CTracker(const CTracker&) = delete;
  CTracker(CTracker&&) = delete;
  CTracker& operator=(const CTracker&) = delete;
  CTracker& operator=(CTracker&&) = delete;

  ~CTracker(void) = default;

  void Update(const std::vector<Point_t>& centers, cv::UMat currFrame, float fps) override;

  bool CanGrayFrameToTrack() const override;
  bool CanColorFrameToTrack() const override;
  size_t GetTracksCount() const override;
  void GetTracks(std::vector<TrackingObject>& tracks) const override;
  void GetRemovedTracks(std::vector<track_id_t>& trackIDs) const override;

 private:
  TrackerSettings m_settings;

  tracks_t m_tracks;  // vector<CTrack>

  track_id_t m_nextTrackID;
  std::vector<track_id_t> m_removedObjects;

  cv::UMat m_prevFrame;

  std::unique_ptr<ShortPathCalculator> m_SPCalculator;  // 匹配问题计算，我们这里只用了匈牙利
  void CreateDistaceMatrix(const std::vector<Point_t>& centers, 
                           distMatrix_t& costMatrix,
                           track_t maxPossibleCost, 
                           track_t& maxCost);
  void UpdateTrackingState(const std::vector<Point_t>& centers, cv::UMat currFrame, float fps);
};

/**
 * @brief: Manage tracks: create, remove, update.
 *         根据setting来创建一个Ctracker,管理ctracks
 */
CTracker::CTracker(const TrackerSettings& settings) : m_settings(settings) {
  m_SPCalculator.reset();  //销毁内部对象并接受新的对象的所有权(如果使用缺省参数的话，也就是没有任何对象的所有权, 此时仅将内部对象释放, 并置为空)
  SPSettings spSettings = {settings.m_distThres, 12};
  // 创建一个匈牙利算法
  m_SPCalculator = std::make_unique<SPHungrian>(spSettings);
  assert(m_SPCalculator);  // 检查是否创建成功
}

///
/// \brief CanGrayFrameToTrack
/// \return
///
bool CTracker::CanGrayFrameToTrack() const {
  bool needColor =
      (m_settings.m_lostTrackType == tracking::LostTrackType::TrackGOTURN) || (m_settings.m_lostTrackType == tracking::LostTrackType::TrackDAT) ||
      (m_settings.m_lostTrackType == tracking::LostTrackType::TrackSTAPLE) || (m_settings.m_lostTrackType == tracking::LostTrackType::TrackLDES);
  return !needColor;
}

///
/// \brief CanColorFrameToTrack
/// \return
///
bool CTracker::CanColorFrameToTrack() const { return true; }

///
/// \brief GetTracksCount
/// \return
///
size_t CTracker::GetTracksCount() const { return m_tracks.size(); }

///
/// \brief GetTracks
/// \return
///
void CTracker::GetTracks(std::vector<TrackingObject>& tracks) const {
  tracks.clear();

  if (m_tracks.size() > tracks.capacity()) tracks.reserve(m_tracks.size());
  for (const auto& track : m_tracks) {
    tracks.emplace_back(track->ConstructObject());
  }
}

///
/// \brief GetRemovedTracks
/// \return
///
void CTracker::GetRemovedTracks(std::vector<track_id_t>& trackIDs) const {
  trackIDs.assign(std::begin(m_removedObjects), std::end(m_removedObjects));
}

///
/// \param centers
/// \param currFrame
/// \param fps
///
void CTracker::Update(const std::vector<Point_t>& centers, cv::UMat currFrame, float fps) {
  m_removedObjects.clear();  // 清空
  UpdateTrackingState(centers, currFrame, fps);
  currFrame.copyTo(m_prevFrame);
}

#define DRAW_DBG_ASSIGNMENT 0

/**
 * @brief 更新track的状态
//  * @param regions    传进来的区域(如果我们是跟踪一个点，那么不用传进来区域啊)  我们传进来的应该是自己设置的 target
 * @param center     我们跟踪点，传进来一个点就ok了
 * @param currFrame  传进来当前帧，图片(如果我们是跟踪一个点，那么不用传进来图片啊)
 * @param fps        帧率
 * 1. 分配
 * 2. 添加track
 * 3. 卡尔曼更新
 */
void CTracker::UpdateTrackingState(const std::vector<Point_t>& centers, cv::UMat currFrame, float fps) {
  // m_tracks是已经跟踪到的
  const size_t N = m_tracks.size();  // Tracking objects
  const size_t M = centers.size();   // Detections of centers

  // 送进来的需要匹配的区域，vector初始化为N个1
  assignments_t assignment(N, -1);  // Assignments centers -> tracks

  // 如果之前匹配结果不是空的
  if (!m_tracks.empty()) {
    // 用于匹配的距离矩阵 N 个 配对 M 个
    distMatrix_t costMatrix(N * M);
    // 可能track到的最大的
    const track_t maxPossibleCost = static_cast<track_t>(currFrame.cols * currFrame.rows);
    track_t maxCost = 0;
    // 计算距离矩阵
    CreateDistaceMatrix(centers, costMatrix, maxPossibleCost, maxCost);

    // Solving assignment problem (shortest paths)
    m_SPCalculator->Solve(costMatrix, N, M, assignment, maxCost);

    // clean assignment from pairs with large distance
    for (size_t i = 0; i < assignment.size(); i++) {
      if (assignment[i] != -1)  //找到了一个region和它匹配
      {
        // 如果距离大于设定值，那么这帧相当于没有找到匹配
        if (costMatrix[i + assignment[i] * N] > m_settings.m_distThres) {
          assignment[i] = -1;
          m_tracks[i]->SkippedFrames()++;
        }
      } else  // 这个就是真没有找到匹配
      {
        // If track have no assigned detect, then increment skipped frames counter.
        m_tracks[i]->SkippedFrames()++;
      }
    }

    // If track didn't get detects long time, remove it.
    for (size_t i = 0; i < m_tracks.size();) {
      if (m_tracks[i]->SkippedFrames() > m_settings.m_maximumAllowedSkippedFrames || m_tracks[i]->IsOutOfTheFrame() ||
          m_tracks[i]->IsStaticTimeout(cvRound(fps * (m_settings.m_maxStaticTime - m_settings.m_minStaticTime)))) {
        m_removedObjects.push_back(m_tracks[i]->GetID());
        m_tracks.erase(m_tracks.begin() + i);
        assignment.erase(assignment.begin() + i);
      } else {
        ++i;
      }
    }
  }

  // Search for unassigned detects and start new tracks for them.
  // 初始时就是所有的都没有track，一样用
  for (size_t i = 0; i < centers.size(); ++i) {
    //  https://blog.csdn.net/huangyimin/article/details/6133650
    //  查找第i个,如果是end，说明是一个新的
    if (find(assignment.begin(), assignment.end(), i) == assignment.end()) {
      // 创建一个ctrack，存起来
      m_tracks.push_back(std::make_unique<CTrack>(centers[i], m_settings.m_kalmanType, m_settings.m_dt, m_settings.m_accelNoiseMag,
                                                  m_settings.m_useAcceleration, m_nextTrackID, m_settings.m_filterGoal == tracking::FilterRect,
                                                  m_settings.m_lostTrackType));

      m_nextTrackID = m_nextTrackID.NextID();// track_id 加 1
    }
  }

  // Update Kalman Filters state
  const ptrdiff_t stop_i = static_cast<ptrdiff_t>(assignment.size());
#pragma omp parallel for //https://blog.csdn.net/weixin_39568744/article/details/88576576
  // 做好的匹配assignment的数量
  for (ptrdiff_t i = 0; i < stop_i; ++i) {
    // If track updated less than one time, than filter state is not correct.
    if (assignment[i] != -1)  // If we have assigned detect, then update using its coordinates,
    {
      // CTrack
      m_tracks[i]->SkippedFrames() = 0;
      // 传center进来
      m_tracks[i]->Update(centers[assignment[i]], true, m_settings.m_maxTraceLength, m_prevFrame, currFrame,
                          m_settings.m_useAbandonedDetection ? cvRound(m_settings.m_minStaticTime * fps) : 0, m_settings.m_maxSpeedForStatic);
    } else{  // if not continue using predictions
      // 这是没有找到匹配的
      Point_t tmp_point;
      tmp_point.x = 0.;
      tmp_point.y = 0.;      
      m_tracks[i]->Update(tmp_point, false, m_settings.m_maxTraceLength, m_prevFrame, currFrame, 0, m_settings.m_maxSpeedForStatic);
    }
  }
}

///
/// \param centers
/// \param costMatrix
/// \param maxPossibleCost
/// \param maxCost
///
void CTracker::CreateDistaceMatrix(const std::vector<Point_t>& centers,
                                   distMatrix_t& costMatrix, track_t maxPossibleCost, track_t& maxCost) {
  const size_t N = m_tracks.size();  // Tracking objects
  maxCost = 0;

  for (size_t i = 0; i < N; ++i) {
    const auto& track = m_tracks[i];

    // Calc predicted area for track
    cv::Size_<track_t> minRadius;
    minRadius.width = m_settings.m_minAreaRadiusPix;
    minRadius.height = m_settings.m_minAreaRadiusPix;
    cv::RotatedRect predictedArea = track->CalcPredictionEllipse(minRadius);

    // Calc distance between track and centers
    for (size_t j = 0; j < centers.size(); ++j) {
      const auto& reg = centers[j];

      auto dist = maxPossibleCost;
        dist = 0;
        size_t ind = 0;
        // Euclidean distance between centers
        if (m_settings.m_distType[ind] > 0.0f && ind == tracking::DistCenters) {
          // 检查这个center在不在范围内  
          track_t ellipseDist = track->IsInsideArea(reg, predictedArea);
          if (ellipseDist > 1)
            dist += m_settings.m_distType[ind];
          else
            dist += ellipseDist * m_settings.m_distType[ind];
        }
        ++ind;

        ++ind;

        ++ind;

        ++ind;

        ++ind;
        assert(ind == tracking::DistsCount);  //几种距离都算过了

      costMatrix[i + j * N] = dist;
      if (dist > maxCost) maxCost = dist;
    }
  }
}

///
/// BaseTracker::CreateTracker 父类静态函数
///
std::unique_ptr<BaseTracker> BaseTracker::CreateTracker(const TrackerSettings& settings) { return std::make_unique<CTracker>(settings); }
