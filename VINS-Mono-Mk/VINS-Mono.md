

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

#### TF关系
\world -> \body -> \camera \

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
  * FeatureTracker::undistortedPoints() 将所有特征点转换到一个标准化平面并且进行畸变
  * 发送图像帧

#### IMU
**IMU measurement model** \
Technical Report VINS_Mono 公式(17)
$$
\hat \alpha _{b_{k+1}}^{b_{k}} = q_{c0}^{b_k}(s(\bar p_{b_{k+1}^{c_0}} - p_{b_{k}^{c_0}})) + \frac{1}{2} g^{c0} \Delta t_{k}^{2} - v_{b_{k+1}}^{c0} \Delta t_{k} \\
\hat \beta _{b_{k+1}}^{b_{k}} = q_{c0}^{b_k} (v_{b_{k+1}}^{c0} + g^{c0} \Delta t_{k} - v_{b_{k}}^{c0})
$$
