## v1.0.1 @2022.5.20 ##
* 1. 添加ORB特征提取模块，只对单张图片提取FAST特征，没有使用金字塔，没有计算灰度质心，没有计算描述子
实验现象:
    可以检测到ORB特征，看天花板时特征数量在400左右，看办公室场景时特征数量可达到上限1000

## v1.0.2 @2022.5.23 ##
* 1. 添加金字塔模块
    金字塔是为了解决尺度问题, 需要注意的地方：
    * a. 按照金字塔面积分配特征点数量
    * b. 每层金字塔的特征是分开的，octtree都不一样
* 2. 计算灰度质心(就是给KeyPoint添加了angle数据)    
    OpenCV的KeyPoint:
    https://docs.opencv.org/4.5.4/d2/d29/classcv_1_1KeyPoint.html#a507d41b54805e9ee5042b922e68e4372
实验现象:
    不同金字塔层数的特征点数确实是按照scalefactor进行分配的
    特征点不稳定，总跳动
    使用金字塔解决尺度问题不是很靠谱？金子塔的高层就是相当于模拟了从远处看当前视野内景物的效果，但是这个金字塔并不连续，也是离散的，只能缓解一下尺度问题
    pyramid_7上根本没看到有60个点啊，是点的坐标不对么？
    ```
    if (level != 0) {
        keypoint->pt *= scale;
      }
    ```
    这段代码进行了坐标转换

## v1.0.3 @2022.5.23 ##
* 1. 计算描述子
    这里提取特征时并没有进行高斯模糊，是在计算描述子前进行的高斯模糊
    描述子是由keypoint附近16x16的圆形区域里按照pattern选取256对点，计算每一对点的灰度大小进行编码，没八个点对编成一组 uchar 8位编码，
    共256个这样的编码组成一个keypoint的描述子，因为计算过程中加入了angle数据，所以可以编码旋转，区域是圆形的，所以有旋转不变性



## v1.1.0 @2022.5.23 ##
* 1. 添加Frame结构，只保留数据存储功能，只保留RGB-D的相关代码,其他的先注释掉
    * a. scale相关变量，都是直接从 orbextractor 里拷贝过来的
    * b. 奥比相机的 imDepth 和 RGB 图像没有对齐，需要重新计算，使得可以根据 RGB 的像素坐标直接查找到对应的深度
    TODO: Frame 里有 MapPoint 成员，所以需要移植 MapPoint (先把tracking 的 work frame 添加进来)


## v1.2.0 @2022.5.24 ##
* 1. 添加 tracking 线程
    * a. tracking 的构造需要一系列参数，这里需要创建一个 setting 类
        robox 里也有类似的 Options 类，使用 jason parser 解析 json 文件
        ORB_SLAM3 用的 cv::FIleStorage 存储并来解析 yaml 文件
        当然还可以用别的例如 protobuf ?? 一些序列化的工具都可以用来存这些配置
    * b. 一个比较重要的成员 GeometricCamera, 暂时注释掉
        这个类里封装好了许多相机相关的函数，例如三角化，极线搜索等，确实封装起来比较好，我们先不用，用的时候再加进来
    * c. 相机参数等都是通过 tracking 传给 Frame 的
        setting --> tracking --> Frame
    * d. Settings 里有位姿信息，所以需要李代数表示，添加了 Sophus 库
       https://www.guyuehome.com/34708

## v1.2.1 @2022.5.25 ##
* 1. 添加 GeometricCamera
    几何相机里封装了一些相机模型相关的方法，可以提高代码复用率
    * a. 添加 g2o 库，因为几何相机里要用到
    * b. 添加 Converter 在 g2o sophus opencv 之间的相互转换
        四元数 q 表示一个旋转 theta 度， q' 表示旋转 theta/2 度，作用在 v 上得到 v'， v' = q'vq'*
        q的左右乘法都可以用矩阵表示 Lq Rq -->  v' = Lq'·Rq'·v
        https://krasjet.github.io/quaternion/quaternion.pdf
        四元数好处省内存空间
    * c. 添加 GeometricTools, 这个函数在2视图重建时使用到了
        TODO: 这里这个三角化涉及 Eigen 的用法
        https://zhuanlan.zhihu.com/p/51835837
    * d. 添加 TwoViewReconstruction
        Reconstruct 这个需要 DBoW, 先注释掉
    * e. 添加 Pinhole 
        这里涉及很多单视图、多视图几何的原理

## v1.2.2 @2022.5.25 ##
* 1. 激活 Setting 里的 GeometricCamera 相关数据
    * a. GeometricCamera 指针 calibration1_ originalCalib1_
    * b. 畸变系数 vPinHoleDistorsion1_
    * c. 加载函数 readCamera1
    PS: 
    * 1.需要把 GeometricCamera 里的 Reconstruct 注释掉，否则这个纯虚函数不实现，Pinhole就无法实例化
    * 2.GeometricCamera 实例化只需要传入相机内参，内参就是相机的一切
    * 3.畸变系数为什么不放在 GeometricCamera 里呢？
* 2. 激活 Tracking 里的 GeometricCamera 相关数据
    只是在 newParameterLoader 里进行了初始化
* 3. 激活 Frame 里的几何相机, 位姿相关数据结构(sophus)
* 4. 激活 tracking 的主函数 GrabImageRGBD
    * a. depth 图像在传进来之前就转好格式了，在 oni_camera 里完成的
* 5. 激活 track() 函数,完成倒帧结构 (currentFrame, lastFrame)
    * a. mState==NO_IMAGES_YET
    * b. mState==NOT_INITIALIZED
        进行初始化
    * c. mState==INITIALIZED
        进行追踪(先为空)
* 6. 取消点云显示
* 7. 显示当前帧的去畸变关键点 mvKeysUn

## v1.2.3 @2022.5.26 ##
* 1. 添加MapPoint
    * a. 只保留 世界位姿 和 描述子 数据，其他注释掉
    * b. 只保留构一个从3D位姿的造函数
    * c. 需要Frame的位姿，这里先把Frame的位姿都设置为(0,0,0)
         Frame的位姿是在track()里进行设置的，初始为0，如果进行了特征匹配则是优化得到的值
* 2. 激活TrackWithMotionModel 
    * a. 修复了Frame拷贝构造函数里没有复制 pCamera , mvpMapPoints 的bug
    * b. 因为流程不同，我们这里 UpdateLastFrame 里其实是给 currentFrame 生成地图点
    * c. 激活viewer线程，显示地图点

## v1.2.4 @2022.5.26 ##
* 1. 激活恒速模型，为帧间追踪做准备
* 2. UpdateLastFrame 恢复为给 LastFrame 创建地图点
* 3. 添加 DBoW2
* 3. Frame 里相应修改
    * a. Frame 里的 mvbOutlier 激活，这个用来记录是否是图优化的外点
    * b. ComputeImageBounds 激活，用来判断重投影的点是否在图像内
    * c. 获取区域内的特征 GetFeaturesInArea
* 4. 添加 ORBmatcher
    * a. 这里用到了 SearchByProjection 把相关函数都激活
    * b. 双目相机时会对右目进行匹配，这里删掉了
    * c. radius = th * scale 搜索范围，阈值 th = 15 还要根据处于哪层金字塔进行改变
    * d. 搜索范围投影范围内的特征点，计算描述子的汉明距离(这个计算函数挺有意思的)
* 5. 添加 Optimizer (没有构造函数)
    * a. 我们这里只用了一个优化函数 PoseOptimization(Frame* pFrame)
            顶点是 VertexSE3Expmap
            边是   EdgeSE3ProjectXYZOnlyPose 
            把其他没用到的类型都注释掉
    * b. 加入 OptimizableTypes 里边是自己定义的一些优化变量，四个比较有用的接口
        ::read // 基类 Edge 的一个纯虚函数，必须实现，其实没有使用，用的是 setMeasurement setInformation
        ::write // 同上
        ::computeError // 这里定义待优化的 loss, 本例中是 重投影误差
        ::linearizeOplus // 这里是优化用的数学公式，每太弄明白
    * c. 优化流程
        1. 构造优化图并输入数据
        2. 进行迭代
* 6. 优化解BUG
    * a. Sophus::SE3f 是用一个四元数加一个平移向量组成 [q, t], 所以初始位姿为 [0, 0, 0, 1, 0, 0, 0]
    * b. optimizer 只要addVertex就会死
    * c. 正常 StereoInitialization 时会创建第一个关键帧，之后 TrackReferenceKeyFrame 时会用这个关键帧来匹配，生成地图点
         我们这里没有关键帧，所以第一帧 mCurrentFrame 里没有地图点，需要生成
         这样还不如第一帧直接跳过，不进行 poseOptimization
    * d. 要保证不进行优化，也可以运行，不能崩溃，因为原版ORBSLAM就不会崩溃
    * e. Optimizer的奇怪结构
            为什么弄成一个class? 因为这样可以通过 EIGEN_MAKE_ALIGNED_OPERATOR_NEW 进行内存对齐(CPU SSE 机制)
            为什么要定义成static函数？ 因为不想 new 一个optimizer对象出来，直接使用其中函数
            那么问题来了， 那个Eigen的宏岂不是没用了
            把这个类结构取消，同样会段错误
    * f. 百度了一下最终解决了问题：
            https://blog.csdn.net/gls_nuaa/article/details/122106358
            https://blog.csdn.net/torresergio/article/details/103253538
        CMakeLists.txt里有一个编译选项 -march=native ，可以根据CPU架构进行优化，g2o编译的时候使用了这个选项，所以我们的工程里也要使用这个选项
* 7. 优化了什么？
        非线性优化就是解一个最小二乘，目标函数是"重投影误差"，根本没有涉及两帧之间的关系，都是当前帧自己玩
        这里分为单目和双目(这个双目是自己构造出来的，RGBD相机其实只有一个目，具体看 ComputeStereoFromRGBD)
        单目时优化的是 Frame1 上的 3D MapPoint 投影到 Frame1 上的像素坐标 (x2,y2) 与 特征提取时 得到的 Frame1 上该特征点的像素坐标 (x1,y1)的差值
        双目时优化的是 Frame1 上的 3D MapPoint 投影到 Frame1 上的像素坐标 (x2,y2) 与 特征提取时 得到的 Frame1 上该特征点的像素坐标 (x1,y1)的差值
                     附加一个在模拟出来的双目里的x坐标差值 ( mvuRight 里保存的那个值)
* 8. 修改了 TrackReferenceKeyFrame 里关于判定为外点的 MapPoint 的观测性质，因为地图等元素还没加入，这些性质不起作用
    