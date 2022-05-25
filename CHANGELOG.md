v1.0.1 @2022.5.20
1. 添加ORB特征提取模块，只对单张图片提取FAST特征，没有使用金字塔，没有计算灰度质心，没有计算描述子
实验现象:
    可以检测到ORB特征，看天花板时特征数量在400左右，看办公室场景时特征数量可达到上限1000

v1.0.2 @2022.5.23
1. 添加金字塔模块
    金字塔是为了解决尺度问题, 需要注意的地方：
    a. 按照金字塔面积分配特征点数量
    b. 每层金字塔的特征是分开的，octtree都不一样
2. 计算灰度质心(就是给KeyPoint添加了angle数据)    
    OpenCV的KeyPoint:
    https://docs.opencv.org/4.5.4/d2/d29/classcv_1_1KeyPoint.html#a507d41b54805e9ee5042b922e68e4372
实验现象:
    不同金字塔层数的特征点数确实是按照scalefactor进行分配的
    特征点不稳定，总跳动
    使用金字塔解决尺度问题不是很靠谱？金子塔的高层就是相当于模拟了从远处看当前视野内景物的效果，但是这个金字塔并不连续，也是离散的，只能缓解一下尺度问题
    pyramid_7上根本没看到有60个点啊，是点的坐标不对么？
    if (level != 0) {
        keypoint->pt *= scale;
      }
    这段代码进行了坐标转换

v1.0.3 @2022.5.23
1. 计算描述子
    这里提取特征时并没有进行高斯模糊，是在计算描述子前进行的高斯模糊
    描述子是由keypoint附近16x16的圆形区域里按照pattern选取256对点，计算每一对点的灰度大小进行编码，没八个点对编成一组 uchar 8位编码，
    共256个这样的编码组成一个keypoint的描述子，因为计算过程中加入了angle数据，所以可以编码旋转，区域是圆形的，所以有旋转不变性



v1.1.0 @2022.5.23
1. 添加Frame结构，只保留数据存储功能，只保留RGB-D的相关代码,其他的先注释掉
    a. scale相关变量，都是直接从 orbextractor 里拷贝过来的
    b. 奥比相机的 imDepth 和 RGB 图像没有对齐，需要重新计算，使得可以根据 RGB 的像素坐标直接查找到对应的深度
    TODO: Frame 里有 MapPoint 成员，所以需要移植 MapPoint (先把tracking 的 work frame 添加进来)


v1.2.0 @2022.5.24
1. 添加 tracking 线程
    a. tracking 的构造需要一系列参数，这里需要创建一个 setting 类
        robox 里也有类似的 Options 类，使用 jason parser 解析 json 文件
        ORB_SLAM3 用的 cv::FIleStorage 存储并来解析 yaml 文件
        当然还可以用别的例如 protobuf ?? 一些序列化的工具都可以用来存这些配置
    b. 一个比较重要的成员 GeometricCamera, 暂时注释掉
        这个类里封装好了许多相机相关的函数，例如三角化，极线搜索等，确实封装起来比较好，我们先不用，用的时候再加进来
    c. 相机参数等都是通过 tracking 传给 Frame 的
        setting --> tracking --> Frame
    d. Settings 里有位姿信息，所以需要李代数表示，添加了 Sophus 库
       https://www.guyuehome.com/34708

v1.2.1 @2022.5.25
1. 添加 GeometricCamera
    几何相机里封装了一些相机模型相关的方法，可以提高代码复用率
    a. 添加 g2o 库，因为几何相机里要用到
    b. 添加 Converter 在 g2o sophus opencv 之间的相互转换
        四元数 q 表示一个旋转 theta 度， q' 表示旋转 theta/2 度，作用在 v 上得到 v'， v' = q'vq'*
        q的左右乘法都可以用矩阵表示 Lq Rq -->  v' = Lq'·Rq'·v
        https://krasjet.github.io/quaternion/quaternion.pdf
    c. 添加 GeometricTools, 这个函数在2视图重建时使用到了
        TODO: 这里这个三角化涉及 Eigen 的用法
        https://zhuanlan.zhihu.com/p/51835837
    d. 添加 TwoViewReconstruction
        Reconstruct 这个需要 DBoW, 先注释掉
    e. 添加 Pinhole 
        这里涉及很多单视图、多视图几何的原理

v1.2.2 @2022.5.25
1. 激活 Setting 里的 GeometricCamera 相关数据
    a. GeometricCamera 指针 calibration1_ originalCalib1_
    b. 畸变系数 vPinHoleDistorsion1_
    c. 加载函数 readCamera1
    PS: 
    1.需要把 GeometricCamera 里的 Reconstruct 注释掉，否则这个纯虚函数不实现，Pinhole就无法实例化
    2.GeometricCamera 实例化只需要传入相机内参，内参就是相机的一切
    3.畸变系数为什么不放在 GeometricCamera 里呢？
2. 激活 Tracking 里的 GeometricCamera 相关数据
    只是在 newParameterLoader 里进行了初始化
