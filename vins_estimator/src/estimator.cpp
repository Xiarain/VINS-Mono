#include "estimator.h"

Estimator::Estimator(): f_manager{Rs}
{
    ROS_INFO("init begins");
    clearState();
    failure_occur = 0;
}

void Estimator::setParameter()
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
    }
    f_manager.setRic(ric);
    ProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
}

void Estimator::clearState()
{
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    solver_flag = INITIAL;
    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();
    relocalize = false;
    retrive_data_vector.clear();
    relocalize_t = Eigen::Vector3d(0, 0, 0);
    relocalize_r = Eigen::Matrix3d::Identity();

    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();
}

/**
 * @brief  对IMU测量数据进行处理
 * @param dt 单位周期时间
 * @param linear_acceleration 线加速度测量值
 * @param angular_velocity 角速度测量值
 */
void Estimator::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    // 对第一帧IMU数据的处理
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    // 预积分
    if (!pre_integrations[frame_count])
    {
        // IntegrationBase 类
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0)
    {
        // 把IMU数据中的单位时间、线加速度和角加速度添加到IntegrationBase预积分类中
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        //if(solver_flag != NON_LINEAR)

        // 临时IntegrationBase类
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        // 向buffer中添加单位时间、线加速度、角速度
        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;

        // 上一次的线加速度
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;

        // 上一次的角加速度和当前时刻的角加速度
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];

        // 更新旋转矩阵
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix(); // delta q

        // 当前时刻的线加速度
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

        // 更新IMU位置和速度
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

/**
 * @brief  处理一帧图像中的特征点
 * @param image 当前帧所有的齐次坐标系下的2D特征点(id号，相机号，特征点位置)
 * @param header ROS消息帧头
 */
void Estimator::processImage(const map<int, vector<pair<int, Vector3d>>> &image, const std_msgs::Header &header)
{
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());

    // FeatureManager f_manager
    // marginalization_flag表示marginalize方式有两种，如果第二最新帧是关键帧的话，那么这个关键帧就会留在滑动窗口中，
    // 时间最长的一帧和其测量值就会被边缘化掉；如果第二最新帧不是关键帧的话，则把这帧的视觉测量舍弃掉而保留IMU测量值在滑动窗口中
    if (f_manager.addFeatureCheckParallax(frame_count, image))
        marginalization_flag = MARGIN_OLD; // 当前帧添加为关键帧则flag置为MARGIN_OLD，值为0
    else
        marginalization_flag = MARGIN_SECOND_NEW; // 否则为MARGIN_SECOND_NEW，值为1

    ROS_DEBUG("this frame is--------------------%s", marginalization_flag ? "reject" : "accept");
    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    ImageFrame imageframe(image, header.stamp.toSec());
    imageframe.pre_integration = tmp_pre_integration;

    // 插入全部帧
    all_image_frame.insert(make_pair(header.stamp.toSec(), imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    // 计算IMU与相机旋转的内参，
    // ESTIMATE_EXTRINSIC 2：没有外部先验矫正，纯粹靠系统内部校准
    if(ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            // 得到两帧之间特征点关系
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;

            // 得camera与IMU之间的旋转偏移常量
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);

                // 初始化得到camera外部旋转量
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    // 系统是否进行初始化
    if (solver_flag == INITIAL)
    {
        if (frame_count == WINDOW_SIZE)
        {
            bool result = false;

            // ESTIMATE_EXTRINSIC == 1
            if( ESTIMATE_EXTRINSIC != 2 && (header.stamp.toSec() - initial_timestamp) > 0.1)
            {
               // 视觉结构初始化,零偏,重力加速度,尺度,速度估计
               result = initialStructure();
               initial_timestamp = header.stamp.toSec();
            }
            if(result)
            {
                // 初始化结束
                solver_flag = NON_LINEAR;

                // 里程计
                solveOdometry();

                // 滑动窗口
                slideWindow();
                f_manager.removeFailures();
                ROS_INFO("Initialization finish!");
                last_R = Rs[WINDOW_SIZE];
                last_P = Ps[WINDOW_SIZE];
                last_R0 = Rs[0];
                last_P0 = Ps[0];
                
            }
            else
                slideWindow(); // slideWindow和frame_count 是同步的，
        }
        else
            frame_count++;
    } // 正常进行视觉里程计
    else
    {
        TicToc t_solve;
        solveOdometry();
        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        // 检测是否SLAM系统失败
        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }

        TicToc t_margin;
        slideWindow();
        f_manager.removeFailures();
        ROS_DEBUG("marginalization costs: %fms", t_margin.toc());
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
    }
}

/**
 * @brief 视觉结构初始化
 * @return
 */
bool Estimator::initialStructure()
{
    TicToc t_sfm;
    //check imu observibility
    // 通过重力协方差检测IMU的可观测性
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;

            // IMU速度预积分
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }

        // 平均值
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);

        // 协方差
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;

            // 协方差
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            //cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        //ROS_WARN("IMU variation %f!", var);
        if(var < 0.25)
        {
            ROS_INFO("IMU excitation not enough!");
            //return false;
        }
    }
    // global sfm
    // 全局SFM
    //  Q: Rqcw
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;

    // 特征点
    // 将每一个3D特征点和相应的2D特征点封装到SFMFeature这个数据结构中
    for (auto &it_per_id : f_manager.feature)
    {
        int imu_j = it_per_id.start_frame - 1;

        // 一个SFMFeature是由一个3D点特征，若干个2D点特征组成
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;

        // 每一个特征点所对应的空间3D点，对应的图像坐标系中的2D是多个，也就是能被多个相机观测到
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            Vector3d pts_j = it_per_frame.point;

            // 一个空间3D特征点所有被相机观测到的2D点，以及出现的帧号
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    } 
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;

    // 选择两帧视差较大、特征点较多的图像帧，利用五点法恢复相对旋转和平移量
    // l为选择的帧号
    if (!relativePose(relative_R, relative_T, l))
    {
        ROS_INFO("Not enough features or parallax; Move device around");
        return false;
    }


    GlobalSFM sfm;

    // SFM构造,初始化初始帧中的相机位置和特征点空间3D位置
    // 输出Q: Rwc
    if(!sfm.construct(frame_count + 1, Q, T, l,
              relative_R, relative_T,
              sfm_f, sfm_tracked_points))
    {
        ROS_DEBUG("global SFM failed!");
        marginalization_flag = MARGIN_OLD;
        return false;
    }

    //solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin();

    // 所有帧
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if((frame_it->first) == Headers[i].stamp.toSec())
        {
            frame_it->second.is_key_frame = true;

            // RIC 相机坐标系转换到IMU坐标系
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if((frame_it->first) > Headers[i].stamp.toSec())
        {
            i++;
        }

        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix(); // Q: Rwc T:twc
        Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);

        // 旋转矢量到旋转矩阵
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            //cout << "feature id " << feature_id;
            for (auto &i_p : id_pts.second)
            {
                //cout << " pts image_frame " << (i_p.second.head<2>() * 460 ).transpose() << endl;
                it = sfm_tracked_points.find(feature_id);
                if(it != sfm_tracked_points.end())
                {
                    // 3D点
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);

                    // 2D点
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);     
        if(pts_3_vector.size() < 6)
        {
            //cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            ROS_DEBUG("Not enough points for solve pnp !");
            return false;
        }

        // PnP求解
        // reve t, Rcw, tcw
        if (! cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            ROS_DEBUG("solve pnp fail!");
            return false;
        }

        cv::Rodrigues(rvec, r);


        MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose(); // Rwc
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp); // twc

        // 将图像坐标系变换到IMU坐标系
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }
    // cameara与IMU对齐
    if (visualInitialAlign())
        return true;
    else
    {
        ROS_INFO("misalign visual structure with IMU");
        return false;
    }

}

/**
 * @brief camera与IMU对齐
 * @return
 */
bool Estimator::visualInitialAlign()
{
    TicToc t_g;
    VectorXd x;
    //solve scale
    // camera与IMU对齐
    // x:输出对齐后的速度、重力加速度、尺度因子
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if(!result)
    {
        ROS_DEBUG("solve g failed!");
        return false;
    }

    // change state
    for (int i = 0; i <= frame_count; i++)
    {
        // 经过camera与IMU对齐优化后，每一个图像帧中相机的旋转和平移量
        // 并且把这些初始图像帧都设置为关键帧
        Matrix3d Ri = all_image_frame[Headers[i].stamp.toSec()].R;
        Vector3d Pi = all_image_frame[Headers[i].stamp.toSec()].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i].stamp.toSec()].is_key_frame = true;
    }

    // 清零f_manager中特征点队列深度信息
    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < dep.size(); i++)
        dep[i] = -1;
    f_manager.clearDepth(dep);

    //triangulat on cam pose , no tic
    Vector3d TIC_TMP[NUM_OF_CAM];
    for(int i = 0; i < NUM_OF_CAM; i++)
        TIC_TMP[i].setZero();

    ric[0] = RIC[0];
    f_manager.setRic(ric);

    // 根据新得到的相机位置进行三角化
    // TIC_TMP[0]值为0；
    // RIC[0] IMU与camera的外参
    f_manager.triangulate(Ps, &(TIC_TMP[0]), &(RIC[0]));

    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }

    // TODO
    // 与第一帧对齐,第一帧就是单位阵
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);

    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;

    // 经过IMU与camera对齐后求解的每一帧的相机速度
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if(frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3); // 之前做矩阵处理的时候这里有一个旋转的逆
        }
    }

    // 尺度因子
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth *= s;
    }

    // 通过实际重力加速度和理想重力加速度(0,0,1)旋转求得旋转矩阵
    Matrix3d R0 = Utility::g2R(g);


    // TODO 这里减去yaw角做了两次处理?
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g = R0 * g;
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
    ROS_DEBUG_STREAM("g0     " << g.transpose());
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose()); 

    return true;
}

/**
 * @brief 在滑动窗口中选择两帧有足够多特征点和视差的帧，利用五点法恢复相对旋转和平移量
 * @param relative_R 输出恢复的旋转量
 * @param relative_T 输出恢复的平移量
 * @param l 输出选择的帧号
 * @return
 */
bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    // 在滑动窗口中选择
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);

        // 两帧图像之间有足够多的特征点
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;

            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());

            // 两帧图像之间有足够多的视差
            // 通过五点法求解两个图像之间的R|t关系
            if(average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
        }
    }
    return false;
}

/**
 * @brief
 */
void Estimator::solveOdometry()
{
    // 帧数需要达到窗口大小
    if (frame_count < WINDOW_SIZE)
        return;

    // 初始化结束
    if (solver_flag == NON_LINEAR)
    {
        TicToc t_tri;
        f_manager.triangulate(Ps, tic, ric);
        ROS_DEBUG("triangulation costs %f", t_tri.toc());
        optimization();
    }
}

/**
 * @brief 数据类型转换，系统位置Ps，q存储在para_Pose；系统速度Vs，陀螺仪零偏Bas，加速度零偏Bgs存储在para_SpeedBias；
 *        IMU与camera外参旋转量tic，平移量q存储在para_Ex_Pose；
 *        特征点深度f_manager.getDepthVector()存在在para_Feature
 */
void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        para_SpeedBias[i][0] = Vs[i].x();
        para_SpeedBias[i][1] = Vs[i].y();
        para_SpeedBias[i][2] = Vs[i].z();

        para_SpeedBias[i][3] = Bas[i].x();
        para_SpeedBias[i][4] = Bas[i].y();
        para_SpeedBias[i][5] = Bas[i].z();

        para_SpeedBias[i][6] = Bgs[i].x();
        para_SpeedBias[i][7] = Bgs[i].y();
        para_SpeedBias[i][8] = Bgs[i].z();
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);
}

/**
 * @brief 系统数据类型转换，通过闭环检测得到最新一个关键帧之前跟踪得到的位姿与闭环检测得到的位姿得到矫正量
 *
 */
void Estimator::double2vector()
{
    // 滑动窗口中第一帧
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    // last_R0，last_P0：滑动窗口的第一帧的位姿
    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }

    // para_Pose[0]是在Estimator::vector2double()函数中储存了滑动窗口中的机体在世界坐标系中的位姿
    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                      para_Pose[0][3],
                                                      para_Pose[0][4],
                                                      para_Pose[0][5]).toRotationMatrix());
    // 通过全局优化得到的yaw轴修正量，如果没有使用闭环检测的话，则仅仅是优化得到的
    double y_diff = origin_R0.x() - origin_R00.x();
    //TODO
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));

    // 进行滑动窗口的机体位姿的Yaw轴方向修正
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {

        Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
        Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                para_Pose[i][1] - para_Pose[0][1],
                                para_Pose[i][2] - para_Pose[0][2]) + origin_P0;
        Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                    para_SpeedBias[i][1],
                                    para_SpeedBias[i][2]);

        Bas[i] = Vector3d(para_SpeedBias[i][3],
                          para_SpeedBias[i][4],
                          para_SpeedBias[i][5]);

        Bgs[i] = Vector3d(para_SpeedBias[i][6],
                          para_SpeedBias[i][7],
                          para_SpeedBias[i][8]);
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d(para_Ex_Pose[i][0],
                          para_Ex_Pose[i][1],
                          para_Ex_Pose[i][2]);
        ric[i] = Quaterniond(para_Ex_Pose[i][6],
                             para_Ex_Pose[i][3],
                             para_Ex_Pose[i][4],
                             para_Ex_Pose[i][5]).toRotationMatrix();
    }

    VectorXd dep = f_manager.getDepthVector();

    // 取特征点的深度信息
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);

    // 是否进行闭环以及是否发生了闭环，利用最新一次的关键帧闭环得到矫正信息
    if (LOOP_CLOSURE && relocalize && retrive_data_vector[0].relative_pose && !retrive_data_vector[0].relocalized)
    {
        for (int i = 0; i < (int)retrive_data_vector.size();i++)
            retrive_data_vector[i].relocalized = true;

        Matrix3d vio_loop_r;
        Vector3d vio_loop_t;
        vio_loop_r = rot_diff * Quaterniond(retrive_data_vector[0].loop_pose[6], retrive_data_vector[0].loop_pose[3], retrive_data_vector[0].loop_pose[4], retrive_data_vector[0].loop_pose[5]).normalized().toRotationMatrix();
        vio_loop_t = rot_diff * Vector3d(retrive_data_vector[0].loop_pose[0] - para_Pose[0][0],
                                retrive_data_vector[0].loop_pose[1] - para_Pose[0][1],
                                retrive_data_vector[0].loop_pose[2] - para_Pose[0][2]) + origin_P0;
        Quaterniond vio_loop_q(vio_loop_r);
        double relocalize_yaw;

        // 由最近一次闭环得到的矫正信息
        relocalize_yaw = Utility::R2ypr(retrive_data_vector[0].R_old).x() - Utility::R2ypr(vio_loop_r).x();
        relocalize_r = Utility::ypr2R(Vector3d(relocalize_yaw, 0, 0));
        relocalize_t = retrive_data_vector[0].P_old- relocalize_r * vio_loop_t;
    }
}
/**
 * @brief SLAM系统失败检测
 * @return
 */
bool Estimator::failureDetection()
{
    if (f_manager.last_track_num < 2)
    {
        ROS_INFO(" little feature %d", f_manager.last_track_num);
        return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
        ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }
    /*
    if (tic(0) > 1)
    {
        ROS_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }
    */
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
        ROS_INFO(" big translation");
        return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
        ROS_INFO(" big z translation");
        return true; 
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
        ROS_INFO(" big delta_angle ");
        return true;
    }
    return false;
}

/**
 * @brief 优化
 */
void Estimator::optimization()
{
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);

    // 添加参数块
    // Technical Report VINS-Mono 公式(20) 中x0 ...... xn,其中x为机体坐标系的位置，速度，旋转，IMU的陀螺仪零偏，加速度零偏
    // IMU和camera外参
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        // PoseLocalParameterization:局部参数化
        // para_Pose[i] 系统位置Ps:旋转量和位移量
        // class PoseLocalParameterization : public ceres::LocalParameterization 继承了ceres的类
        // PoseLocalParameterization中定义了Ps的加法和雅克比矩阵
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);

        // para_SpeedBias：系统速度，陀螺仪零偏，加速度零偏
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }

    // camera与IMU的外参
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        // para_Ex_Pose：IMU和camera的外参，包括旋转量和平移量
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);

        // 是否固定外参，不进行优化
        if (!ESTIMATE_EXTRINSIC)
        {
            ROS_DEBUG("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
        else
            ROS_DEBUG("estimate extinsic param");
    }

    TicToc t_whole, t_prepare;

    // 数据类型转换
    vector2double();

    // 添加系统残差
    // 添加先验残差
    if (last_marginalization_info)
    {
        // construct new marginlization_factor
        // 残差
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }

    // 添加IMU残差值
    // 添加IMU的测量值，其中为相邻两帧的位置，速度，加速度计的零偏，陀螺仪的零偏
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)
            continue;
        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);

        // Problem::AddResidualBlock
        // imu_factor： cost_function class; NULL: loss_function; para_Pose[i]等后续形参：parameter_blocks;
        // para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]:IMUFactor::Evaluate()，
        // imu_factor类继承了CostFunction类，重新设计虚函数Evaluate();
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);

    }

    int f_m_cnt = 0;
    int feature_index = -1;

    // 添加camera残差值
    // 遍历特征点库中每一个3D特征点，
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();

        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
 
        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        // 遍历每个特征点的被观测到的图像特征点，imu_i这个3D特征点第一次在i帧中被观测到，imu_j这个特征点在后续j帧继续被观测到
        // 3D特征点在i帧中的图像特征点位置pts_i,在j帧中的图像特征点位置pts_j
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i == imu_j)
            {
                continue;
            }
            Vector3d pts_j = it_per_frame.point;
            ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);

            // para_Feature[feature_index]：每个3D特征点的深度；
            // para_Pose[i] body坐标系Ps:旋转量和位移量
            // para_Ex_Pose：IMU和camera的外参，包括旋转量和平移量
            problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]);
            f_m_cnt++;
        }
    }
    relocalize = false;

    //loop close factor
    // 闭环检测
    if(LOOP_CLOSURE)
    {
        int loop_constraint_num = 0;

        // 遍历所有的闭环
        for (int k = 0; k < (int)retrive_data_vector.size(); k++)
        {    
            for(int i = 0; i < WINDOW_SIZE; i++)
            {
                if(retrive_data_vector[k].header == Headers[i].stamp.toSec())
                {
                    relocalize = true;
                    ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();

                    // retrive_data_vector[k]:vector<RetriveData>;
                    // retrive_data_vector[k].loop_pose:通过闭环关键帧求解出来的世界坐标系,被优化的对象
                    problem.AddParameterBlock(retrive_data_vector[k].loop_pose, SIZE_POSE, local_parameterization);

                    loop_window_index = i;
                    loop_constraint_num++;
                    int retrive_feature_index = 0;
                    int feature_index = -1;

                    // 遍历特征点库中的所有的关键点
                    for (auto &it_per_id : f_manager.feature)
                    {
                        it_per_id.used_num = it_per_id.feature_per_frame.size();

                        // 这个特征点被两个观测到，且被开始观测到的位置应该要早于最近两帧
                        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                            continue;

                        ++feature_index;

                        // star：这个3D特征点被观测到第一帧序号
                        // 遍历所有的特征点，找到
                        int start = it_per_id.start_frame;

                        // 在滑动窗口中，在这个特征点出现的关键帧以及之后的关键帧
                        if(start <= i)
                        {   
                            while(retrive_data_vector[k].features_ids[retrive_feature_index] < it_per_id.feature_id)
                            {
                                retrive_feature_index++;
                            }

                            if(retrive_data_vector[k].features_ids[retrive_feature_index] == it_per_id.feature_id)
                            {
                                // 新关键帧得到的特征点
                                Vector3d pts_j = Vector3d(retrive_data_vector[k].measurements[retrive_feature_index].x, retrive_data_vector[k].measurements[retrive_feature_index].y, 1.0);

                                // 特征点库中的检索得到的特征点
                                Vector3d pts_i = it_per_id.feature_per_frame[0].point;
                                
                                ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);

                                // para_Pose[start]：相机位置, retrive_data_vector[k].loop_pose：形成闭环帧两帧之间的相对位置关系,
                                // para_Ex_Pose[0]：IMU与camera的外参, para_Feature[feature_index]：每个特征点的深度；
                                problem.AddResidualBlock(f, loss_function, para_Pose[start], retrive_data_vector[k].loop_pose, para_Ex_Pose[0], para_Feature[feature_index]);
                                //problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]);
                                retrive_feature_index++;
                            }     
                        }
                    }
                            
                }
            }
        }
        ROS_DEBUG("loop constraint num: %d", loop_constraint_num);
    }
    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    ROS_DEBUG("prepare for ceres: %f", t_prepare.toc());

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;

    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //cout << summary.BriefReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    ROS_DEBUG("solver costs: %f", t_solver.toc());

    // relative info between two loop frame
    // 计算滑动窗口中与每一个闭环关键帧的相对位姿
    if(LOOP_CLOSURE && relocalize)
    {
        // 遍历所有的闭环关键帧
        for (int k = 0; k < (int)retrive_data_vector.size(); k++)
        {
            for(int i = 0; i< WINDOW_SIZE; i++)
            {
                if(retrive_data_vector[k].header == Headers[i].stamp.toSec())
                {
                    retrive_data_vector[k].relative_pose = true;

                    // 滑动窗口中的每一帧的位姿
                    Matrix3d Rs_i = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
                    Vector3d Ps_i = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);

                    Quaterniond Qs_loop;

                    Qs_loop = Quaterniond(retrive_data_vector[k].loop_pose[6],  retrive_data_vector[k].loop_pose[3],  retrive_data_vector[k].loop_pose[4],  retrive_data_vector[k].loop_pose[5]).normalized().toRotationMatrix();

                    // 闭环关键帧的位姿
                    Matrix3d Rs_loop = Qs_loop.toRotationMatrix();
                    Vector3d Ps_loop = Vector3d( retrive_data_vector[k].loop_pose[0],  retrive_data_vector[k].loop_pose[1],  retrive_data_vector[k].loop_pose[2]);

                    // 修正检测到闭环两帧之间的相对位姿
                    retrive_data_vector[k].relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
                    retrive_data_vector[k].relative_q = Rs_loop.transpose() * Rs_i;
                    retrive_data_vector[k].relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());
                    if (abs(retrive_data_vector[k].relative_yaw) > 30.0 || retrive_data_vector[k].relative_t.norm() > 20.0)
                        retrive_data_vector[k].relative_pose = false;
                        
                }
            } 
        } 
    }

    // 系统数据类型转换，通过闭环检测得到最新一个关键帧的之前跟踪得到的位姿与闭环检测得到的位姿做差得到矫正量
    double2vector();

    TicToc t_whole_marginalization;

    // 当前帧添加为关键帧则flag置为MARGIN_OLD
    // 如果第二最新帧是关键帧的话，那么这个关键帧就会留在滑动窗口中，时间最长的一帧和其测量值就会被边缘化掉
    // 如果第二最新帧不是关键帧的话，则把这帧的视觉测量舍弃掉而保留IMU测量值在滑动窗口中
    // 这样的策略会保证系统的稀疏性
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();

        // 数据类型转换
        vector2double();

        // last_marginalization_info为MarginalizationInfo
        if (last_marginalization_info)
        {
            vector<int> drop_set;

            // 已经被边缘化掉的关键帧
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                // para_Pose[0]:滑动窗口中时间最长的关键帧
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            // MarginalizationFactor类继承了ceres::CostFunction
            // !所有带factor的类都继承了ceres::CostFunction
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);

            marginalization_info->addResidualBlockInfo(residual_block_info);
            //  marginalization_info->addResidualBlockInfo(residual_block_info)
            //      residual_block_info（marginalization_factor）
            //          marginalization_factor（last_marginalization_info）
        }

        // 添加IMU测量值，IMU误差模型
        {
            // 如果这个IMU预积分时间小于阀值
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                           vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        // camera测量残差
        {
            int feature_index = -1;

            // 特征点
            // 遍历每个特征点的被观测到的图像特征点，imu_i这个3D特征点第一次在i帧中被观测到，imu_j这个特征点在后续j帧继续被观测到
            // 3D特征点在i帧中的图像特征点位置pts_i,在j帧中的图像特征点位置pts_j
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();

                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if (imu_i == imu_j)
                        continue;

                    Vector3d pts_j = it_per_frame.point;
                    ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                   vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]},
                                                                                   vector<int>{0, 3});
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());
        
        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        // 滑动窗口中参数块存储地址调整
        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        // 更新最新关键帧
        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
        
    }
    else // 如果第二最新帧不是关键帧的话，则把这帧的视觉测量舍弃掉（边缘化）而保留IMU测量值在滑动窗口中
    {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();

            // 数据转换
            vector2double();

            if (last_marginalization_info)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    // para_SpeedBias[WINDOW_SIZE - 1] 最新的关键帧
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);

                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");

            // 开始边缘化
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());
            
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    // reinterpret_cast：强制类型转换符，从指针类型到一个足够大的整数类型
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

            // 参数块
            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
            
        }
    }
    ROS_DEBUG("whole marginalization costs: %f", t_whole_marginalization.toc());
    
    ROS_DEBUG("whole time for ceres: %f", t_whole.toc());
}

/**
 * @brief 滑动窗口，维持滑动窗口的大小，保证SLAM运行计算的复杂度
 *        如果第二最新帧是关键帧的话，那么这个关键帧就会留在滑动窗口中，时间最长的一帧和其测量值就会被边缘化掉
 *        如果第二最新帧不是关键帧的话，则把这帧的视觉测量舍弃掉而保留IMU测量值在滑动窗口中,这样的策略会保证系统的稀疏性
 *        (0, 1, ..., N)关键帧，0是时间最长的关键帧，N是最新关键帧
 */
void Estimator::slideWindow()
{
    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD)
    {
        // Rs[0] 滑动窗口中存在时间最长的关键帧
        back_R0 = Rs[0];
        back_P0 = Ps[0];

        // 把第窗口中的关键帧往前挪一个位置，也就是
        if (frame_count == WINDOW_SIZE)
        {
            // 把滑动窗口中的0号关键帧以及IMU测量值移动到N号关键帧
            // 0 1 2 3 ... N
            // 1 2 3 ... N 0
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Rs[i].swap(Rs[i + 1]);

                std::swap(pre_integrations[i], pre_integrations[i + 1]);

                dt_buf[i].swap(dt_buf[i + 1]);
                linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                Headers[i] = Headers[i + 1];
                Ps[i].swap(Ps[i + 1]);
                Vs[i].swap(Vs[i + 1]);
                Bas[i].swap(Bas[i + 1]);
                Bgs[i].swap(Bgs[i + 1]);
            }

            // 第N-1帧放到第N帧
            // 1 2 3 ... N N 没有第0帧了
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
            Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
            Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

            // 1 2 3 ... N 空
            delete pre_integrations[WINDOW_SIZE];
            // 把最新的测量值给WINDOW_SIZE位置上的预积分容器
            // acc_0,gyr_0 主要是为了后续的中值积分使用
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            // 清除时间最长的关键帧相应的IMU测量值
            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            if (true || solver_flag == INITIAL)
            {
                double t_0 = Headers[0].stamp.toSec();
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                all_image_frame.erase(all_image_frame.begin(), it_0);

            }
            slideWindowOld();
        }
    }
    else // 边缘化新的数据 frame_count == 10
    {
        if (frame_count == WINDOW_SIZE)
        {
            // 遍历最新帧对应的的所有IMU数据
            // 如果第二最新帧不是关键帧的话，则把这帧的视觉测量舍弃掉,将IMU数据积分到上一帧中
            for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
            {
                // 把这个IMU测量值单位时间，线加速度，角加速度都放到预积分的类中，也就是接受了这次IMU的测量值
                double tmp_dt = dt_buf[frame_count][i];
                Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                // 将丢弃的图像帧对应的IMU数据放入到frame_count - 1预积分数据中
                pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                dt_buf[frame_count - 1].push_back(tmp_dt);
                linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
            }

            // 第N帧放到第N-1帧
            // 把最新关键帧数据给倒数第二新的帧数据
            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Vs[frame_count - 1] = Vs[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];
            Bas[frame_count - 1] = Bas[frame_count];
            Bgs[frame_count - 1] = Bgs[frame_count];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            slideWindowNew();
        }
    }
}

// real marginalization is removed in solve_ceres()
void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
}
// real marginalization is removed in solve_ceres()
void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();

}

