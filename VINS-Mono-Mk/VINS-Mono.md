

### 系统启动命令
$roslaunch vins_estimator euroc.launch \
$roslaunch vins_estimator vins_rviz.launch \
$rosbag play YOUR_PATH_TO_DATASET/MH_03_medium.bag \
$roslaunch benchmark_publisher publish.launch  sequence_name:=MH_03_medium

### 文件目录概述
* ar_demo
* benchmark_publisher 发布数据集中参考值
* config
* camera_model
  * calib
  * camera_models
    * PinholeCamera.c 针孔相机模型
    * ScaramuzzaCamera.c SCARAMUZZA相机模型，
    * CataCamera.c MEI相机模型
    * EquidistantCamera.c KANNALA_BRANDT相机模型
    * CostFunctionFactory.c 优化相机模型中的参数
  * chessboard 用于检测棋盘格特征点
  * gpl
  * sparse_graph
  * intrinsic_calib.cc 相机矫正模块主函数
* feature_trackers
  * feature_tracker_node(main()函数,ROS接受图像的回调函数)
  * feature_tracker.c 特征点跟踪的具体实现
* support_files
* vins_estimator
  * src
     * factor
     * initial
       * initial_sfm
       * solve_5pts
       * initial_ex_rotation
     * loop_closure
     * utility
        * 显示
  * estimator_node(main()函数)
    * 多线程 measurement_process、loop_detection、 pose_graph

#### 重要变量说明
Ps：世界坐标系下的平移量,Rs：世界坐标系下的旋转量,Vs:世界坐标系的速度量 \
ric、tic：IMU与camera之间的外参 \
estimator.f_manager.feature: 全局特征点

#### TF关系
\world -> \body -> \camera \
\body坐标系为IMU坐标系

#### 系统结构
![](picture\1.png)

![](picture\2.png)

##### 相机矫正过程

图像矫正可通过两种方式执行，分别为正向矫正和逆向矫正[参考博客](http://www.cnblogs.com/dzyBK/p/5579206.html)。正向矫正是通过畸变坐标算出标准坐标，而逆向矫正是通过标准坐标算出畸变坐标。
Opencv中UndistortPoints就是执行的正向矫正过程，而initUndistortRectifyMap执行的是逆向矫正过程。\
正向矫正的流程为：畸变像素坐标→畸变物理坐标→标准物理坐标→标准像素坐标。\
逆向矫正的流程为：标准像素坐标→标准物理坐标→畸变物理坐标→畸变像素坐标。
* initUndistortRectifyMap()
  * spaceToPlane()
    * distortion()
##### 特征点跟踪过程
* readParameters() 通过ROS来读取参数
* FeatureTracker::readIntrinsicParameter() 读取相机内参
* 读取mask图片（鱼眼相机）
* img_callback() ROS回调函数
  * FeatureTracker::readImage()
    * cv::createCLAHE() 直方图均衡化（可选）
    * cv::calcOpticalFlowPyrLK() LK金字塔光流法(同时会去除那些无法追踪到的特征点)
    * FeatureTracker::rejectWithF() 通过F矩阵去除外点
    * FeatureTracker::setMask() 设置遮挡部分（鱼眼相机）
    * cv::goodFeaturesToTrack() 添加角点(第一帧初始化特征点检测也是通过这里完成的)
    * FeatureTracker::addPoints() 添加新检测到的特征点
  * FeatureTracker::undistortedPoints() 将所有特征点转换到一个归一化平面并且进行畸变
  * 发送图像帧
##### estimator_node
* readParameters() 通过ROS来读取参数
* Estimator::setParameter()
* registerPub() 用于RVIZ显示的Topic  (visualization.cpp)
* 订阅IMU、特征点和camera原始图像Topic消息，回调函数分别为 imu_callback()、feature_callback()和raw_image_callback()
* 多线程处理 process()、loop_detection()、 pose_graph()
* 闭环处理（可选）

**process线程**
* send_imu()
  * Estimator::processIMU() IMU预积分
    * IntegrationBase::push_back()
      * IntegrationBase::propagate()
        * IntegrationBase::midPointIntegration()计算雅克比矩阵 jacobian = F*jacobian
* Estimator::processImage() 处理图像
  * FeatureManager::addFeatureCheckParallax() 通过检测两帧之间的视差决定是否作为关键帧
    * FeatureManager::compensatedParallax2() 对相近的特征点进行视差计算
  * 是否初始化camera与IMU之间的外参
    * FeatureManager::getCorresponding() 得到两帧之间特征点关系
    * InitialEXRotation::CalibrationExRotation() 得到camera与IMU之间的旋转偏移常量
    * Estimator::initialStructure() 视觉结构初始化
       * 通过重力协方差检测IMU的可观测性
       * Estimator::relativePose() 选择两个视差较大的帧，利用五点法恢复相对旋转和平移量
       * GlobalSFM::construct() 全局SFM初始化全部初始帧中的相机位置和特征点空间3D位置
         * 通过五点法求解的3D点，利用PnP算法求解相机位置，然后三角化得到更多的空间3D点
         * 全局BA优化相机位置、3D特征点
       * 利用PnP算法进行相机位置求解、更新
       * Estimator::visualInitialAlign() cameara与IMU对齐
         * VisualIMUAlignment() 对齐camera与IMU (initial_aligmentc.cpp)
           * VisualIMUAlignment()  IMU陀螺仪零偏初始化
           * LinearAlignment() 速度，重力向量，尺度初始化
         * FeatureManager::triangulate() 三角化
         * 更新相机速度，位置和旋转量(通过精确求解的尺度，重力向量)
    * Estimator::solveOdometry()
      * FeatureManager::triangulate() 三角化
      * Estimator::optimization()
        * 添加先验残差，IMU测量残差，camera测量残差，注意这里IMU项和camera项之间是有一个系数，这个系数就是他们各自的协方差矩阵
        * 边缘化处理，如果第二最新帧是关键帧的话，那么这个关键帧就会留在滑动窗口中，时间最长的一帧和其测量值就会被边缘化掉；如果第二最新帧不是关键帧的话，则把这帧的视觉测量舍弃掉而保留IMU测量值在滑动窗口中
        * 全局闭环检测优化 $\min\limits_{\chi,q_{v}^{w},p_{v}^{w}}\{ \|r_p - H_p \chi \|^{2} + \sum_{k \in \mathcal{B}} \| r_{\mathcal{B}} (\hat z _{b_{k+1}}^{b_{k}},\chi)  \}$
* 是否进行闭环(可系统设置)
    * 清除estimator.retrive_data_vector前一个闭环，添加最新retrive feature（retrive_data_buf）
    * 如果经过前面检测，最新第二帧是关键帧，则将这帧添加到关键帧队列中
      * KeyFrame::setExtrinsic() 设置IMU与camera的外参
      * KeyFrame::buildKeyFrameFeatures() 将空间的3D点构建当前关键帧的特征点
      * 添加到关键帧队列中（keyframe_buf）
      * 检查闭环是否出错
        * 两个匹配帧之间yaw角度过大或者是平移量过大，则认为是匹配错误，移除此次闭环匹配
        * 将此次闭环检测添加到位图优化缓冲区optimize_posegraph_buf
 * 给RVIZ发送里程计、关键位置、相机位置、点云和TF关系
 * update() 更新IMU系统参数
   * predict() 通过IMU的测量值进行tmp_Q，tmp_P，tmp_V预测更新


**loop_detection线程**
* process_loop_detection()
  * LoopClosure::initCameraModel() 初始化相机模型
  * KeyFrame::extractBrief() 在前端视觉追踪过程中提取的特征点对于闭环检测是不够的，所以需要提取更加多的特征点以及相应的描述子
  * LoopClosure::startLoopClosure() 开始闭环检测
    * demoDetector<TVocabulary, TDetector, TDescriptor>::run()
      * TemplatedLoopDetector<TDescriptor, F>::detectLoop()
  * 是否闭环成功
    * KeyFrame::findConnectionWithOldFrame() 利用描述子匹配关键帧库中与当前关键帧的特征点，并且用PnP算法求解这两帧之间的关系
      * KeyFrame::searchByDes() 关键帧库中匹配上的关键帧特征点与当前帧上的特征点进行匹配
        * KeyFrame::searchInAera() 通过一个描述子对一个描述子向量进行匹配，得到最高得分的特征点
      * KeyFrame::FundmantalMatrixRANSAC() 利用opencv中的findFundamentalMat函数利用RANSAC进行基础矩阵求解，排除外点
      * KeyFrame::PnPRANSAC() 利用RANSAC算法求解PnP问题，得到关键帧库中关键帧与当前关键帧的位置，并且删除外点
      * 向retrived feature缓冲区（retrive_data_buf）中添加新的数据
      * KeyFrameDatabase::addLoop() 向关键帧数据库中添加闭环序号,更新闭环显示
        * CameraPoseVisualization::add_loopedge() 更新关键帧库中闭环显示
      * PoseGraph显示更新
  * 如果关键帧库中帧数过大，则减低采样，删除那些位置和角度关键帧密集的关键帧，保留位置和角度有一定间隔的关键帧

**pose_graph线程**

##### IMU
**IMU measurement model** \
Technical Report VINS_Mono 公式(17)
$$
\hat \alpha _{b_{k+1}}^{b_{k}} = q_{c0}^{b_k}(s(\bar p_{b_{k+1}^{c_0}} - p_{b_{k}^{c_0}})) + \frac{1}{2} g^{c0} \Delta t_{k}^{2} - v_{b_{k+1}}^{c0} \Delta t_{k}) \\
\hat \beta _{b_{k+1}}^{b_{k}} = q_{c0}^{b_k} (v_{b_{k+1}}^{c0} + g^{c0} \Delta t_{k} - v_{b_{k}}^{c0})
$$

在IMU中将误差状态合并到标量状态中：
$$
R \approx \hat R \cdot (I + [\delta \theta]_{\times})
$$
$$
q = \hat q \otimes \delta q, \ \delta q \approx
\begin{bmatrix}
\frac{1}{2} \delta \theta \\
1
\end{bmatrix}
$$
注意这个四元数定义的顺序。
#####注意事项
1. std::thread \
std::mutex \
std::future \
std::condition_variable
2.  OpenCV求解PnP问题需要主要的地方：
$$
\begin{bmatrix}
R & t \\
0 & 1 \\
\end{bmatrix} ^{-1}
=
\begin{bmatrix}
\begin{bmatrix}
I & t \\
0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
R & 0 \\
0 & 1 \\
\end{bmatrix}
\end{bmatrix}^{-1}
=
\begin{bmatrix}
\begin{bmatrix}
R & 0 \\
0 & 1 \\
\end{bmatrix}^{-1}
\begin{bmatrix}
I & t \\
0 & 1 \\
\end{bmatrix}^{-1}
\end{bmatrix} \\
=
\begin{bmatrix}
\begin{bmatrix}
R^T & 0 \\
0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
I & -t \\
0 & 1 \\
\end{bmatrix}
\end{bmatrix}
=
\begin{bmatrix}
R^T & -R^Tt \\
0 & 1 \\
\end{bmatrix}
$$
- 通过PnP求解出来的$R$和$t$，
$R$的第$i$行 表示摄像机坐标系中的第$i$个坐标轴方向的单位向量在世界坐标系里的坐标；
$R$的第$j$列 表示世界坐标系中的第$i$个坐标轴方向的单位向量在摄像机坐标系里的坐标；$t$表示的是世界坐标系的原点在相机坐标系的坐标；$-R^Tt$表示相机坐标系的原点在世界坐标系的坐标。
