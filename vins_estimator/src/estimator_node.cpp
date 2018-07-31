#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"
#include "loop-closure/loop_closure.h"
#include "loop-closure/keyframe.h"
#include "loop-closure/keyframe_database.h"
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

Estimator estimator;

std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
std::mutex m_posegraph_buf;
queue<int> optimize_posegraph_buf;
queue<KeyFrame*> keyframe_buf;
queue<RetriveData> retrive_data_buf;

int sum_of_wait = 0;

std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_loop_drift;
std::mutex m_keyframedatabase_resample;
std::mutex m_update_visualization;
std::mutex m_keyframe_buf;
std::mutex m_retrive_data_buf;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;

queue<pair<cv::Mat, double>> image_buf;
LoopClosure *loop_closure;
KeyFrameDatabase keyframe_database;

int global_frame_cnt = 0;
//camera param
camodocal::CameraPtr m_camera;
vector<int> erase_index;
std_msgs::Header cur_header;
Eigen::Vector3d relocalize_t{Eigen::Vector3d(0, 0, 0)};
Eigen::Matrix3d relocalize_r{Eigen::Matrix3d::Identity()};

/**
 * @brief 通过IMU的测量值进行tmp_Q，tmp_P，tmp_V预测更新
 * @param imu_msg IMU测量值
 */
void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    // 计算单位时间
    double t = imu_msg->header.stamp.toSec();
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    // tmp_Q 四元数 world坐标系到body坐标系
    // acc_0 上一次的线加速度测量值
    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba - tmp_Q.inverse() * estimator.g);

    // gyr_0 上一次的角加速度测量值
    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;

    // 更新四元数
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    // 这里的tmp_Q是经过更新过的当前时刻的四元数
    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba - tmp_Q.inverse() * estimator.g);

    // 上一次的加速度和当前加速度的平均和
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = relocalize_r * estimator.Ps[WINDOW_SIZE] + relocalize_t;
    tmp_Q = relocalize_r * estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());

}

/**
 * @brief 对齐IMU测量数据和camera特征点测量数据
 */
std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;

    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;

        // IMU数据队列中的最后一个数据的时间戳小于特征点队列中第一个数据的时间戳，等待IMU数据
        if (!(imu_buf.back()->header.stamp > feature_buf.front()->header.stamp))
        {
            ROS_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }

        //  IMU数据队列中的第一个数据的时间戳大于特征点队列中第一个数据的时间戳，那么就舍弃一个特征点队列中的数据
        if (!(imu_buf.front()->header.stamp < feature_buf.front()->header.stamp))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<sensor_msgs::ImuConstPtr> IMUs;

        // 取出所有的时间戳小于当前特征点队列的IMU数据
        while (imu_buf.front()->header.stamp <= img_msg->header.stamp)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }

        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    // 把IMU数据存储到buffer中
    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();

    // 消息队列，通知其他线程处理数据
    con.notify_one();

    {
        std::lock_guard<std::mutex> lg(m_state);

        // 通过IMU值跟新tmp_P，tmp_V，tmp_Q
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";

        // 这里的里程计只是IMU的测量值进行计算得出
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    }
}

void raw_image_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    // ROS图像格式与OpenCV格式进行转换
    cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    //image_pool[img_msg->header.stamp.toNSec()] = img_ptr->image;

    // 如果进行闭环检测的话，才进行原始图像处理
    if(LOOP_CLOSURE)
    {
        i_buf.lock();

        // 向缓冲区加入原始图像和时间戳
        image_buf.push(make_pair(img_ptr->image, img_msg->header.stamp.toSec()));
        i_buf.unlock();
    }
}

void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    m_buf.lock();

    // 向缓冲区添加特征点，格式为ROS消息帧
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}

/**
 * @brief 发送
 * @param imu_msg
 */
void send_imu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if (current_time < 0)
        current_time = t;
    double dt = t - current_time;
    current_time = t;

    double ba[]{0.0, 0.0, 0.0};
    double bg[]{0.0, 0.0, 0.0};

    double dx = imu_msg->linear_acceleration.x - ba[0];
    double dy = imu_msg->linear_acceleration.y - ba[1];
    double dz = imu_msg->linear_acceleration.z - ba[2];

    double rx = imu_msg->angular_velocity.x - bg[0];
    double ry = imu_msg->angular_velocity.y - bg[1];
    double rz = imu_msg->angular_velocity.z - bg[2];
    //ROS_DEBUG("IMU %f, dt: %f, acc: %f %f %f, gyr: %f %f %f", t, dt, dx, dy, dz, rx, ry, rz);

    // Estimator::processIMU
    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
}

//thread:loop detection
/**
 * @brief 闭环检测，只是检测闭环帧，但是不进行优化
 *        闭环检测是检测全局关键帧与
 */
void process_loop_detection()
{
    if(loop_closure == NULL)
    {
        // 通过系统开始读取YAML文件获得字典文件目录
        // 字典文件brief_k10L6.bin的路径/VINS-Mono/support_files
        const char *voc_file = VOC_FILE.c_str();
        TicToc t_load_voc;
        ROS_DEBUG("loop start loop");
        cout << "voc file: " << voc_file << endl;

        loop_closure = new LoopClosure(voc_file, IMAGE_COL, IMAGE_ROW);
        ROS_DEBUG("loop load vocbulary %lf", t_load_voc.toc());

        // 初始化相机模型
        loop_closure->initCameraModel(CAM_NAMES);
    }

    // 是否进行闭环检测，可以配置
    while(LOOP_CLOSURE)
    {
        KeyFrame* cur_kf = NULL; 

        m_keyframe_buf.lock();

        // 清空keyframe_buf，在cur_kf中存放keyframe_buf中尾部最后一帧
        while(!keyframe_buf.empty())
        {
            if(cur_kf!=NULL)
                delete cur_kf;

            // 返回头部数据，该关键帧是最新创建的
            cur_kf = keyframe_buf.front();

            // 头部数据出队
            keyframe_buf.pop();
        }
        m_keyframe_buf.unlock();

        if (cur_kf != NULL)
        {
            cur_kf->global_index = global_frame_cnt;

            // 将当前帧cur_kf添加为关键帧库中
            m_keyframedatabase_resample.lock();
            keyframe_database.add(cur_kf);
            m_keyframedatabase_resample.unlock();

            // 当前关键帧
            cv::Mat current_image;
            current_image = cur_kf->image;   

            bool loop_succ = false;
            int old_index = -1;
            vector<cv::Point2f> cur_pts;
            vector<cv::Point2f> old_pts;
            TicToc t_brief;

            // 当前关键帧提取brief特征
            // 在前端视觉追踪过程中提取的特征点对于闭环检测是不够的，所以需要提取更加多的特征点以及相应的描述子
            // 所有的关键帧的特征点和描述子都会添加到window_keypoints,window_descriptors,
            cur_kf->extractBrief(current_image);
            //printf("loop extract %d feature using %lf\n", cur_kf->keypoints.size(), t_brief.toc());
            TicToc t_loopdetect;

            // old_index:匹配上的帧号
            // cur_kf->keypoints,cur_kf->descriptors:当前帧的关键点、描述子
            // old_index：输出查找到的关键帧序号
            // 因为图像是一帧一帧增长的，所以通过这个startLoopClosure，将一帧的特征点和描述子放入关键帧数据库中
            loop_succ = loop_closure->startLoopClosure(cur_kf->keypoints, cur_kf->descriptors, cur_pts, old_pts, old_index);

            double t_loop = t_loopdetect.toc();
            ROS_DEBUG("t_loopdetect %f ms", t_loop);

            // 是否闭环成功
            if(loop_succ)
            {
                // 闭环匹配上的关键帧库中的关键帧
                KeyFrame* old_kf = keyframe_database.getKeyframe(old_index);
                if (old_kf == NULL)
                {
                    ROS_WARN("NO such frame in keyframe_database");
                    ROS_BREAK();
                }
                // global_frame_cnt：全局关键帧计数
                // old_index:在关键帧库中闭环上的关键帧
                ROS_DEBUG("loop succ %d with %drd image", global_frame_cnt, old_index);
                assert(old_index!=-1);
                
                Vector3d T_w_i_old, PnP_T_old;
                Matrix3d R_w_i_old, PnP_R_old;

                old_kf->getPose(T_w_i_old, R_w_i_old);
                std::vector<cv::Point2f> measurements_old;
                std::vector<cv::Point2f> measurements_old_norm;
                std::vector<cv::Point2f> measurements_cur;
                std::vector<int> features_id_matched;

                // 查找两帧之间的关系
                cur_kf->findConnectionWithOldFrame(old_kf, measurements_old, measurements_old_norm, PnP_T_old, PnP_R_old, m_camera);

                measurements_cur = cur_kf->measurements_matched;
                features_id_matched = cur_kf->features_id_matched;
                // send loop info to VINS relocalization
                int loop_fusion = 0;

                if( (int)measurements_old_norm.size() > MIN_LOOP_NUM && global_frame_cnt - old_index > 35 && old_index > 30)
                {

                    Quaterniond PnP_Q_old(PnP_R_old);

                    // 检索特征点存放到队列中
                    RetriveData retrive_data;

                    // 发生闭环的位置的全局关键帧位置
                    retrive_data.cur_index = cur_kf->global_index;
                    retrive_data.header = cur_kf->header;
                    retrive_data.P_old = T_w_i_old; // 关键帧库中关键帧的位置
                    retrive_data.R_old = R_w_i_old;
                    retrive_data.relative_pose = false;
                    retrive_data.relocalized = false;
                    retrive_data.measurements = measurements_old_norm; // 一帧中所有的特征点
                    retrive_data.features_ids = features_id_matched;
                    retrive_data.loop_pose[0] = PnP_T_old.x(); // 通过闭环关键帧求解出来的世界坐标系
                    retrive_data.loop_pose[1] = PnP_T_old.y(); //
                    retrive_data.loop_pose[2] = PnP_T_old.z();
                    retrive_data.loop_pose[3] = PnP_Q_old.x();
                    retrive_data.loop_pose[4] = PnP_Q_old.y();
                    retrive_data.loop_pose[5] = PnP_Q_old.z();
                    retrive_data.loop_pose[6] = PnP_Q_old.w();

                    // 向retrive_data_buf缓冲区添加新的retrived feature
                    // retrive_data_buf,存放的是已经发生闭环的新关键帧相对于世界坐标系的位姿
                    m_retrive_data_buf.lock();
                    retrive_data_buf.push(retrive_data);
                    m_retrive_data_buf.unlock();

                    cur_kf->detectLoop(old_index);
                    old_kf->is_looped = 1;
                    loop_fusion = 1;

                    m_update_visualization.lock();

                    // 向关键帧数据库中添加关键帧闭环
                    keyframe_database.addLoop(old_index);

                    CameraPoseVisualization* posegraph_visualization = keyframe_database.getPosegraphVisualization();
                    pubPoseGraph(posegraph_visualization, cur_header);  
                    m_update_visualization.unlock();
                }


                // visualization loop info
                if(0 && loop_fusion)
                {
                    int COL = current_image.cols;
                    //int ROW = current_image.rows;
                    cv::Mat gray_img, loop_match_img;
                    cv::Mat old_img = old_kf->image;
                    cv::hconcat(old_img, current_image, gray_img);
                    cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
                    cv::Mat loop_match_img2;
                    loop_match_img2 = loop_match_img.clone();
                    /*
                    for(int i = 0; i< (int)cur_pts.size(); i++)
                    {
                        cv::Point2f cur_pt = cur_pts[i];
                        cur_pt.x += COL;
                        cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
                    }
                    for(int i = 0; i< (int)old_pts.size(); i++)
                    {
                        cv::circle(loop_match_img, old_pts[i], 5, cv::Scalar(0, 255, 0));
                    }
                    for (int i = 0; i< (int)old_pts.size(); i++)
                    {
                        cv::Point2f cur_pt = cur_pts[i];
                        cur_pt.x += COL ;
                        cv::line(loop_match_img, old_pts[i], cur_pt, cv::Scalar(0, 255, 0), 1, 8, 0);
                    }
                    ostringstream convert;
                    convert << "/home/tony-ws/raw_data/loop_image/"
                            << cur_kf->global_index << "-" 
                            << old_index << "-" << loop_fusion <<".jpg";
                    cv::imwrite( convert.str().c_str(), loop_match_img);
                    */
                    for(int i = 0; i< (int)measurements_cur.size(); i++)
                    {
                        cv::Point2f cur_pt = measurements_cur[i];
                        cur_pt.x += COL;
                        cv::circle(loop_match_img2, cur_pt, 5, cv::Scalar(0, 255, 0));
                    }
                    for(int i = 0; i< (int)measurements_old.size(); i++)
                    {
                        cv::circle(loop_match_img2, measurements_old[i], 5, cv::Scalar(0, 255, 0));
                    }
                    for (int i = 0; i< (int)measurements_old.size(); i++)
                    {
                        cv::Point2f cur_pt = measurements_cur[i];
                        cur_pt.x += COL ;
                        cv::line(loop_match_img2, measurements_old[i], cur_pt, cv::Scalar(0, 255, 0), 1, 8, 0);
                    }

                    ostringstream convert2;
                    convert2 << "/home/tony-ws/raw_data/loop_image/"
                            << cur_kf->global_index << "-" 
                            << old_index << "-" << loop_fusion <<"-2.jpg";
                    cv::imwrite( convert2.str().c_str(), loop_match_img2);
                }
                  
            }
            //release memory
            cur_kf->image.release();
            global_frame_cnt++;

            if (t_loop > 1000 || keyframe_database.size() > MAX_KEYFRAME_NUM)
            {
                m_keyframedatabase_resample.lock();
                erase_index.clear();

                // 如果关键帧库中帧数过大，则减低采样，删除那些位置和角度关键帧密集的关键帧，保留位置和角度有一定间隔的关键帧
                keyframe_database.downsample(erase_index);
                m_keyframedatabase_resample.unlock();

                if(!erase_index.empty())
                    loop_closure->eraseIndex(erase_index);
            }
        }
        std::chrono::milliseconds dura(10);
        std::this_thread::sleep_for(dura);
    }
}

//thread: pose_graph optimization
void process_pose_graph()
{
    while(true)
    {
        m_posegraph_buf.lock();
        int index = -1;
        while (!optimize_posegraph_buf.empty())
        {
            index = optimize_posegraph_buf.front();
            optimize_posegraph_buf.pop();
        }
        m_posegraph_buf.unlock();
        if(index != -1)
        {
            Vector3d correct_t = Vector3d::Zero();
            Matrix3d correct_r = Matrix3d::Identity();
            TicToc t_posegraph;
            keyframe_database.optimize4DoFLoopPoseGraph(index,
                                                    correct_t,
                                                    correct_r);
            ROS_DEBUG("t_posegraph %f ms", t_posegraph.toc());
            m_loop_drift.lock();
            relocalize_r = correct_r;
            relocalize_t = correct_t;
            m_loop_drift.unlock();


            m_update_visualization.lock();
            keyframe_database.updateVisualization();
            CameraPoseVisualization* posegraph_visualization = keyframe_database.getPosegraphVisualization();
            m_update_visualization.unlock();

            pubOdometry(estimator, cur_header, relocalize_t, relocalize_r);
            pubPoseGraph(posegraph_visualization, cur_header); 
            nav_msgs::Path refine_path = keyframe_database.getPath();
            updateLoopPath(refine_path);
        }

        std::chrono::milliseconds dura(5000);
        std::this_thread::sleep_for(dura);
    }
}

// thread: visual-inertial odometry
void process()
{
    while (true)
    {
        // measurements数据类型 = sensor_msgs::ImuConstPtr + sensor_msgs::PointCloudConstPtr
        // measurement 第一部分是IMU值和第二部分是图像特征点
        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;

        // con.wait() 一直被阻塞，直到有数据发送过来
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0;
                 });
        lk.unlock();

        // 对每一个measurement 进行处理
        for (auto &measurement : measurements)
        {
            // measurement 第一部分是IMU值
            for (auto &imu_msg : measurement.first)
                send_imu(imu_msg);

            // measurement 第二部分是图像特征点
            auto img_msg = measurement.second;
            ROS_DEBUG("processing vision data with stamp %f \n", img_msg->header.stamp.toSec());

            TicToc t_s;

            // map与vector的区别是map有两个属性，而vector只有一个属性
            // image数据类型实际含义：特征点ID号，相机ID号，特征点2D齐次坐标
            map<int, vector<pair<int, Vector3d>>> image;

            // 处理一帧图像中的特征点
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;

                // 由于是齐次坐标系，z值必须为1
                ROS_ASSERT(z == 1);

                // emplace_back可以避免额外类的复制和移动操作
                image[feature_id].emplace_back(camera_id, Vector3d(x, y, z));
            }

            // 处理图像特征点
            estimator.processImage(image, img_msg->header);
            /**
            *** start build keyframe database for loop closure
            **/
            if(LOOP_CLOSURE)
            {
                // remove previous loop
                // 清除estimator.retrive_data_vector前一个闭环
                vector<RetriveData>::iterator it = estimator.retrive_data_vector.begin();
                for(; it != estimator.retrive_data_vector.end(); )
                {
                    if ((*it).header < estimator.Headers[0].stamp.toSec())
                    {
                        it = estimator.retrive_data_vector.erase(it);
                    }
                    else
                        it++;
                }

                m_retrive_data_buf.lock();
                // 如果retrive_data_buf缓冲区不为空，则将如果retrive_data_buf添加到estimator.retrive_data_vector
                // retrive_data_buf缓冲区数据是在闭环检测线程中被添加的，其中就是retrived feature
                while(!retrive_data_buf.empty())
                {
                    RetriveData tmp_retrive_data = retrive_data_buf.front();
                    retrive_data_buf.pop();
                    estimator.retrive_data_vector.push_back(tmp_retrive_data);
                }
                m_retrive_data_buf.unlock();

                //WINDOW_SIZE - 2 is key frame
                // 第二最新帧是关键帧的话，那么这个关键帧就会留在滑动窗口中，时间最长的一帧和其测量值就会被边缘化掉；
                if(estimator.marginalization_flag == 0 && estimator.solver_flag == estimator.NON_LINEAR)
                {
                    // 这一帧相机旋转和位移
                    Vector3d vio_T_w_i = estimator.Ps[WINDOW_SIZE - 2];
                    Matrix3d vio_R_w_i = estimator.Rs[WINDOW_SIZE - 2];

                    i_buf.lock();
                    while(!image_buf.empty() && image_buf.front().second < estimator.Headers[WINDOW_SIZE - 2].stamp.toSec())
                    {
                        // 先进先出，队伍头部图像帧先出
                        image_buf.pop();
                    }
                    i_buf.unlock();
                    //assert(estimator.Headers[WINDOW_SIZE - 1].stamp.toSec() == image_buf.front().second);
                    // relative_T   i-1_T_i relative_R  i-1_R_i
                    // 这个变量没有被使用，没有意义
                    cv::Mat KeyFrame_image;
                    // 返回队列头部数据
                    KeyFrame_image = image_buf.front().first;
                    
                    const char *pattern_file = PATTERN_FILE.c_str();
                    Vector3d cur_T;
                    Matrix3d cur_R;

                    // 通过闭环检测进行当前帧的位置矫正
                    cur_T = relocalize_r * vio_T_w_i + relocalize_t;
                    cur_R = relocalize_r * vio_R_w_i;

                    // 创建关键帧，序号为estimator.Headers[WINDOW_SIZE - 2].stamp.toSec()；
                    KeyFrame* keyframe = new KeyFrame(estimator.Headers[WINDOW_SIZE - 2].stamp.toSec(), vio_T_w_i, vio_R_w_i, cur_T, cur_R, image_buf.front().first, pattern_file);

                    // 设置这个关键帧中的IMU和camera的外参
                    keyframe->setExtrinsic(estimator.tic[0], estimator.ric[0]);

                    // 将空间的3D点构建当前关键帧的特征点
                    keyframe->buildKeyFrameFeatures(estimator, m_camera);

                    // 将keyframe添加到keyframe_buf缓冲区
                    m_keyframe_buf.lock();
                    // 队列尾部添加数据
                    keyframe_buf.push(keyframe);
                    m_keyframe_buf.unlock();

                    // update loop info
                    // estimator.retrive_data_vector不为空，
                    if (!estimator.retrive_data_vector.empty() && estimator.retrive_data_vector[0].relative_pose)
                    {
                        if(estimator.Headers[0].stamp.toSec() == estimator.retrive_data_vector[0].header)
                        {
                            // keyframe_database关键帧数据库
                            KeyFrame* cur_kf = keyframe_database.getKeyframe(estimator.retrive_data_vector[0].cur_index);                            

                            // 如果两个匹配帧之间yaw角度过大或者是平移量过大，则认为是匹配错误
                            if (abs(estimator.retrive_data_vector[0].relative_yaw) > 30.0 || estimator.retrive_data_vector[0].relative_t.norm() > 20.0)
                            {
                                ROS_DEBUG("Wrong loop");
                                cur_kf->removeLoop();
                            }
                            else 
                            {
                                // retrive_data_vector[0],一个关键帧
                                cur_kf->updateLoopConnection( estimator.retrive_data_vector[0].relative_t, 
                                                              estimator.retrive_data_vector[0].relative_q, 
                                                              estimator.retrive_data_vector[0].relative_yaw);

                                // 向位图优化缓冲区添加闭环匹配
                                m_posegraph_buf.lock();
                                optimize_posegraph_buf.push(estimator.retrive_data_vector[0].cur_index);
                                m_posegraph_buf.unlock();
                            }
                        }
                    }
                } //! if(estimator.marginalization_flag == 0 && estimator.solver_flag == estimator.NON_LINEAR)
            } //! if(LOOP_CLOSURE)

            double whole_t = t_s.toc();

            // 将系统数据写入文件中
            printStatistics(estimator, whole_t);
            std_msgs::Header header = img_msg->header;
            header.frame_id = "world";
            cur_header = header;
            m_loop_drift.lock();

            if (estimator.relocalize)
            {
                relocalize_t = estimator.relocalize_t;
                relocalize_r = estimator.relocalize_r;
            }

            // 发送相应的消息到Rviz中显示
            pubOdometry(estimator, header, relocalize_t, relocalize_r);
            pubKeyPoses(estimator, header, relocalize_t, relocalize_r);
            pubCameraPose(estimator, header, relocalize_t, relocalize_r);
            pubPointCloud(estimator, header, relocalize_t, relocalize_r);
            pubTF(estimator, header, relocalize_t, relocalize_r);

            m_loop_drift.unlock();
            //ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());

        }//! for (auto &measurement : measurements)

        m_buf.lock();
        m_state.lock();

        // 更新IMU系统参数
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");

    // rqt_console属于ROS日志框架的一部分，用来显示节点的输出信息
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);
    estimator.setParameter();
#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    // 订阅 IMU、camera特征点、相机原始图像的topic消息
    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_image = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_raw_image = n.subscribe(IMAGE_TOPIC, 2000, raw_image_callback);

    // 多线程 measurement_process、loop_detection pose_graph
    // 测量值处理
    std::thread measurement_process{process};
    std::thread loop_detection, pose_graph;

    // 全局闭环，可以设置
    if (LOOP_CLOSURE)
    {
        ROS_WARN("LOOP_CLOSURE true");
        loop_detection = std::thread(process_loop_detection);   
        pose_graph = std::thread(process_pose_graph);
        m_camera = CameraFactory::instance()->generateCameraFromYamlFile(CAM_NAMES);
    }
    ros::spin();

    return 0;
}
