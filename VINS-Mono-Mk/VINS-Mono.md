

### 系统启动命令
$roslaunch vins_estimator euroc.launch \
$roslaunch vins_estimator vins_rviz.launch \
$rosbag play YOUR_PATH_TO_DATASET/MH_05_difficult.bag \
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

* initUndistortRectifyMap()
  * spaceToPlane()
    * distortion()

**IMU measurement model** \
Technical Report VINS_Mono 公式(17)
$$
\hat \alpha _{b_{k+1}}^{b_{k}} = q_{c0}^{b_k}(s(\bar p_{b_{k+1}^{c_0}} - p_{b_{k}^{c_0}})) + \frac{1}{2} g^{c0} \Delta t_{k}^{2} - v_{b_{k+1}}^{c0} \Delta t_{k} \\
\hat \beta _{b_{k+1}}^{b_{k}} = q_{c0}^{b_k} (v_{b_{k+1}}^{c0} + g^{c0} \Delta t_{k} - v_{b_{k}}^{c0})
$$
