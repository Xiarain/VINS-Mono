#include "keyframe.h"

KeyFrame::KeyFrame(double _header, Eigen::Vector3d _vio_T_w_i, Eigen::Matrix3d _vio_R_w_i, 
                   Eigen::Vector3d _cur_T_w_i, Eigen::Matrix3d _cur_R_w_i, 
                   cv::Mat &_image, const char *_brief_pattern_file)
:header{_header}, image{_image}, BRIEF_PATTERN_FILE(_brief_pattern_file)
{
    T_w_i = _cur_T_w_i;
    R_w_i = _cur_R_w_i;
    COL = image.cols;
    ROW = image.rows;
    use_retrive = false;
    is_looped = 0;
    has_loop = 0;
    update_loop_info = 0;
    vio_T_w_i = _vio_T_w_i;
    vio_R_w_i = _vio_R_w_i;
}

/*****************************************utility function************************************************/
bool inBorder(const cv::Point2f &pt, int COL, int ROW)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

template <typename Derived>
static void reduceVector(vector<Derived> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}
/**
 * @brief 当前关键帧提取Brief描述子，Fast特征点，存放在window_keypoints、window_descriptors
 * @param image 当前关键帧
 */
void KeyFrame::extractBrief(cv::Mat &image)
{
    // opencv库
    BriefExtractor extractor(BRIEF_PATTERN_FILE);

    // measurements：在KeyFrame::buildKeyFrameFeatures()创建关键帧是所追踪到的特征点和描述子
    extractor(image, measurements, keypoints, descriptors);

    // keypoints 是该副图像特征点与measurements特征点之和
    // keypoints.size() - measurements.size() 这样的操作表明就是直接对measurement这部分的特征点进行操作
    int start = keypoints.size() - measurements.size();

    // 将在KeyFrame::buildKeyFrameFeatures()所追踪到的特征点和描述子添加window_keypoints、window_descriptors窗口特征点向量中
    for(int i = 0; i< (int)measurements.size(); i++)
    {
        window_keypoints.push_back(keypoints[start + i]);
        window_descriptors.push_back(descriptors[start + i]);
    }
}
void KeyFrame::setExtrinsic(Eigen::Vector3d T, Eigen::Matrix3d R)
{
    qic = R;
    tic = T;
}

/**
 * @brief 将空间的3D点构建当前关键帧的特征点
 * @param estimator Estimator状态估计器
 * @param m_camera 相机类指针
 */
void KeyFrame::buildKeyFrameFeatures(Estimator &estimator, const camodocal::CameraPtr &m_camera)
{
    // FeaturePerId,空间每一个3D特征点
    for (auto &it_per_id : estimator.f_manager.feature)
    {
        //该3D特征点对应2D特征点数量
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        //if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
        //    continue;

        // 该3D特征点一共出现在图像帧的次数
        int frame_size = it_per_id.feature_per_frame.size();

        // 该3D特征点出现在窗口中
        if(it_per_id.start_frame <= WINDOW_SIZE - 2 && it_per_id.start_frame + frame_size - 1 >= WINDOW_SIZE - 2)
        {
            //features current measurements
            // 特征值当前的测量值
            Vector3d point = it_per_id.feature_per_frame[WINDOW_SIZE - 2 - it_per_id. start_frame].point;
            Vector2d point_uv;

            // 将3D点投影到图像平面上,并且进行矫正
            m_camera->spaceToPlane(point, point_uv);

            // 添加当前帧中2D图像特征点到measurements
            measurements.push_back(cv::Point2f(point_uv.x(), point_uv.y()));

            // 归一化平面
            pts_normalize.push_back(cv::Point2f(point.x()/point.z(), point.y()/point.z()));
            features_id.push_back(it_per_id.feature_id);
            //features 3D pos from first measurement and inverse depth
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;

            // 3D点云
            point_clouds.push_back(estimator.Rs[it_per_id.start_frame] * (qic * pts_i + tic) + estimator.Ps[it_per_id.start_frame]);
        }
    }
}

/**
 * @brief 判断点pt是否在以center为中心的area_size范围中
 * @param pt
 * @param center 中心点
 * @param area_size 范围
 * @return
 */
bool KeyFrame::inAera(cv::Point2f pt, cv::Point2f center, float area_size)
{
    if(center.x < 0 || center.x > COL || center.y < 0 || center.y > ROW)
        return false;
    if(pt.x > center.x - area_size && pt.x < center.x + area_size &&
       pt.y > center.y - area_size && pt.y < center.y + area_size)
        return true;
    else
        return false;
}

/**
 * @brief 通过一个描述子对一个描述子向量进行匹配，得到最高得分的特征点
 * @param center_cur 中心点
 * @param area_size 范围
 * @param window_descriptor 当前关键帧中的一个描述子
 * @param descriptors_old 关键帧库中匹配上的关键帧的描述子
 * @param keypoints_old 关键帧库中匹配上的关键帧的特征点
 * @param best_match 关键帧库中最好匹配的关键点
 * @return
 */
bool KeyFrame::searchInAera(cv::Point2f center_cur, float area_size,
                            const BRIEF::bitset window_descriptor,
                            const std::vector<BRIEF::bitset> &descriptors_old,
                            const std::vector<cv::KeyPoint> &keypoints_old,
                            cv::Point2f &best_match)
{
    cv::Point2f best_pt;

    // brief描述子一般是256字符长度
    int bestDist = 128;
    int bestIndex = -1;

    for(int i = 0; i < (int)descriptors_old.size(); i++)
    {
        // 判断点keypoints_old[i].pt是否在以center_cur为中心的area_size范围中
        if(!inAera(keypoints_old[i].pt, center_cur, area_size))
            continue;

        // 计算两个字符串的汉明距离
        // 查找汉明距离最大的两个字符串
        int dis = HammingDis(window_descriptor, descriptors_old[i]);
        if(dis < bestDist)
        {
            bestDist = dis;
            bestIndex = i;
        }
    }

    // 通过序号返回最好匹配的2d关键点
    if (bestIndex != -1)
    {
      best_match = keypoints_old[bestIndex].pt;
      return true;
    }
    else
      return false;
}

/**
 * @brief 利用opencv中的findFundamentalMat函数利用RANSAC进行基础矩阵求解，排除外点
 * @param measurements_old 通过描述子使特征库关键帧与当前帧匹配上的特征点，特征点属于关键帧库
 * @param measurements_old_norm 输出关键帧库中相应关键帧的特征点
 * @param m_camera 相机类型
 */
void KeyFrame::FundmantalMatrixRANSAC(vector<cv::Point2f> &measurements_old,
                                      vector<cv::Point2f> &measurements_old_norm,
                                      const camodocal::CameraPtr &m_camera)
{
    // 匹配上的特征点需要大于8对
    if (measurements_old.size() >= 8)
    {
        measurements_old_norm.clear();

        vector<cv::Point2f> un_measurements(measurements_matched.size()), un_measurements_old(measurements_old.size());

        for (int i = 0; i < (int)measurements_matched.size(); i++)
        {
            double FOCAL_LENGTH = 460.0;
            Eigen::Vector3d tmp_p;

            // 将2D点投影到标准化平面，并且进行畸变矫正
            // 当前关键帧提取的特征点
            m_camera->liftProjective(Eigen::Vector2d(measurements_matched[i].x, measurements_matched[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_measurements[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            // 关键帧库中相应关键帧的特征点
            m_camera->liftProjective(Eigen::Vector2d(measurements_old[i].x, measurements_old[i].y), tmp_p);

            // measurements_old_norm:投影到归一化平面上，并且进行相机模型矫正
            measurements_old_norm.push_back(cv::Point2f(tmp_p.x()/tmp_p.z(), tmp_p.y()/tmp_p.z()));

            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_measurements_old[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }
        
        vector<uchar> status;

        cv::findFundamentalMat(un_measurements, un_measurements_old, cv::FM_RANSAC, 5.0, 0.99, status);

        // 删除误匹配的特征点
        reduceVector(measurements_old, status);
        reduceVector(measurements_old_norm, status);
        reduceVector(measurements_matched, status);
        reduceVector(features_id_matched, status);
        reduceVector(point_clouds_matched, status);
        
    }
}

/**
 * @brief 关键帧库中匹配上的关键帧特征点与当前帧上的特征点进行匹配
 * @param measurements_old (out) 关键帧库中关键帧与当前帧匹配上的特征点，属于关键帧库中
 * @param measurements_old_norm (out) 在此函数没有进行操作
 * @param descriptors_old 关键帧库中匹配上的关键帧的描述子
 * @param keypoints_old 关键帧库中匹配上的关键帧的特征点
 * @param m_camera 相机类型
 */
void KeyFrame::searchByDes(std::vector<cv::Point2f> &measurements_old,
                           std::vector<cv::Point2f> &measurements_old_norm,
                           const std::vector<BRIEF::bitset> &descriptors_old,
                           const std::vector<cv::KeyPoint> &keypoints_old,
                           const camodocal::CameraPtr &m_camera)
{
    //ROS_INFO("loop_match before cur %d %d, old %d", (int)window_descriptors.size(), (int)measurements.size(), (int)descriptors_old.size());
    std::vector<uchar> status;

    // 当前关键帧的描述子
    for(int i = 0; i < (int)window_descriptors.size(); i++)
    {
        cv::Point2f pt(0.f, 0.f);
        // measurements[i]:当前关键帧提取的特征点; 200：范围
        // window_descriptors[i]: 当前关键帧提取的描述子; descriptors_old：关键帧库中匹配上的关键帧的描述子
        // keypoints_old： 关键帧库中匹配上的关键帧的特征点； pt：最好匹配的2D关键点
        // 通过关键帧库中匹配上的关键帧的描述子跟当前帧描述子向量进行匹配，关键帧库中得到最高得分的特征点pt
        if (searchInAera(measurements[i], 200, window_descriptors[i], descriptors_old, keypoints_old, pt))
          status.push_back(1);
        else
          status.push_back(0);
        measurements_old.push_back(pt);
    }

    // 删除那些匹配不上的特征点
    measurements_matched = measurements;
    features_id_matched = features_id;
    point_clouds_matched = point_clouds;

    reduceVector(measurements_old, status);
    reduceVector(measurements_matched, status);
    reduceVector(features_id_matched, status);
    reduceVector(point_clouds_matched, status);
}

/**
 * @brief 利用RANSAC算法求解PnP问题，得到关键帧库中与当前关键帧的位置,并且删除外点
 * @param measurements_old 关键帧库与当前帧匹配上的特征点,属于关键帧库
 * @param measurements_old_norm
 * @param PnP_T_old 关键帧库中关键帧的旋转
 * @param PnP_R_old 关键帧库中关键帧的平移
 */
void KeyFrame::PnPRANSAC(vector<cv::Point2f> &measurements_old,
                         std::vector<cv::Point2f> &measurements_old_norm, 
                         Eigen::Vector3d &PnP_T_old, Eigen::Matrix3d &PnP_R_old)
{
    cv::Mat r, rvec, t, D, tmp_r;
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
    Matrix3d R_inital;
    Vector3d P_inital;

    // IMU坐标系转换到camera坐标系
    Matrix3d R_w_c = vio_R_w_i * qic;
    Vector3d T_w_c = vio_T_w_i + vio_R_w_i * tic;

    // 将关键帧库中关键帧的旋转、位移，作为PnP求解的初始值，这样求解出来的结果就是相当与世界坐标系下的坐标
    R_inital = R_w_c.inverse();
    P_inital = -(R_inital * T_w_c);
    
    cv::eigen2cv(R_inital, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_inital, t);

    vector<cv::Point3f> pts_3_vector;

    // 当前关键帧3D点云
    for(auto &it: point_clouds_matched)
        pts_3_vector.push_back(cv::Point3f((float)it.x(),(float)it.y(),(float)it.z()));

    cv::Mat inliers;
    TicToc t_pnp_ransac;

    // opencv 版本选择
    // D：畸变系数
    // useExtrinsicGuess当为ture时，这个函数会将rvec和tvec作为旋转和位移初始近似值
    if (C   V_MAJOR_VERSION < 3)
        solvePnPRansac(pts_3_vector, measurements_old_norm, K, D, rvec, t, true, 100, 10.0 / 460.0, 100, inliers);
    else
    {
        if (CV_MINOR_VERSION < 2)
            solvePnPRansac(pts_3_vector, measurements_old_norm, K, D, rvec, t, true, 100, sqrt(10.0 / 460.0), 0.99, inliers);
        else
            solvePnPRansac(pts_3_vector, measurements_old_norm, K, D, rvec, t, true, 100, 10.0 / 460.0, 0.99, inliers);

    }
    ROS_DEBUG("t_pnp_ransac %f ms", t_pnp_ransac.toc());

    std::vector<uchar> status;
    for (int i = 0; i < (int)measurements_old_norm.size(); i++)
        status.push_back(0);

    for( int i = 0; i < inliers.rows; i++)
    {
        int n = inliers.at<int>(i);
        status[n] = 1;
    } 

    cv::Rodrigues(rvec, r);

    Matrix3d R_pnp, R_w_c_old;
    cv::cv2eigen(r, R_pnp);

    // 变换矩阵求逆
    // 通过OpenCV PnP求解出来的变换矩阵，是从世界坐标系的原点在相机坐标系的坐标
    // 所以需要求逆变换为相机坐标系的原点在世界坐标系的坐标
    R_w_c_old = R_pnp.transpose();

    Vector3d T_pnp, T_w_c_old;
    cv::cv2eigen(t, T_pnp);
    T_w_c_old = R_w_c_old * (-T_pnp);

    // 从camera坐标系转换到IMU坐标系
    PnP_R_old = R_w_c_old * qic.transpose();
    PnP_T_old = T_w_c_old - PnP_R_old * tic;   

    reduceVector(measurements_old, status);
    reduceVector(measurements_old_norm, status);
    reduceVector(measurements_matched, status);
    reduceVector(features_id_matched, status);
    reduceVector(point_clouds_matched, status);


}

/**
 * @brief 利用描述子匹配关键帧库中与当前关键帧的特征点，并且用PnP算法求解这两帧之间的关系
 * @param old_kf 关键帧库中匹配上的关键帧
 * @param measurements_old (out) 特征库关键帧中与当前帧匹配上的特征点
 * @param measurements_old_norm (out) 投影到归一化平面上，并且进行相机模型矫正
 * @param PnP_T_old 输出关键帧库关键帧与当前关键帧之间的位移量
 * @param PnP_R_old 输出关键帧库关键帧与当前关键帧之间的旋转量
 * @param m_camera 相机类型
 * @return
 */
bool KeyFrame::findConnectionWithOldFrame(const KeyFrame* old_kf,
                                          std::vector<cv::Point2f> &measurements_old, std::vector<cv::Point2f> &measurements_old_norm,
                                          Eigen::Vector3d &PnP_T_old, Eigen::Matrix3d &PnP_R_old,
                                          const camodocal::CameraPtr &m_camera)
{
    TicToc t_match;

    // 关键帧库中匹配上的关键帧特征点与当前帧上的特征点进行匹配
    // measurements_old_norm:在此函数中没有被操作
    // measurements_old：特征库关键帧中与当前帧匹配上的特征点
    searchByDes(measurements_old, measurements_old_norm, old_kf->descriptors, old_kf->keypoints, m_camera);

    // 利用opencv中的findFundamentalMat函数利用RANSAC进行基础矩阵求解，排除外点
    // measurements_old：特征点属于关键帧库
    FundmantalMatrixRANSAC(measurements_old, measurements_old_norm, m_camera);

    // 经过基础矩阵和RANSAC进行误匹配滤除，匹配上的特征点必须大于阀值
    if ((int)measurements_old_norm.size() > MIN_LOOP_NUM)
    {
        // 利用RANSAC算法求解PnP问题，得到关键帧库中与当前关键帧的位置，并且删除外点
        // PnP_T_old,PnP_R_old,关键帧库中关键帧的在世界坐标系中下的旋转和位移
        PnPRANSAC(measurements_old, measurements_old_norm, PnP_T_old, PnP_R_old);
    }
    ROS_DEBUG("loop final use num %d %lf---------------", (int)measurements_old.size(), t_match.toc());
    return true;
}

void KeyFrame::updatePose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i)
{
    unique_lock<mutex> lock(mMutexPose);
    T_w_i = _T_w_i;
    R_w_i = _R_w_i;
}

void KeyFrame::updateOriginPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i)
{
    unique_lock<mutex> lock(mMutexPose);
    vio_T_w_i = _T_w_i;
    vio_R_w_i = _R_w_i;
}

void KeyFrame::getPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i)
{
    unique_lock<mutex> lock(mMutexPose);
    _T_w_i = T_w_i;
    _R_w_i = R_w_i;
}

void KeyFrame::getOriginPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i)
{
    unique_lock<mutex> lock(mMutexPose);
    _T_w_i = vio_T_w_i;
    _R_w_i = vio_R_w_i;
}

/**
 * @brief
 * @param relative_t
 * @param relative_q
 * @param relative_yaw
 */
void KeyFrame::updateLoopConnection(Vector3d relative_t, Quaterniond relative_q, double relative_yaw)
{
    has_loop = 1;
    update_loop_info = 1;
    unique_lock<mutex> lock(mLoopInfo);
    Eigen::Matrix<double, 8, 1> connected_info;
    connected_info <<relative_t.x(), relative_t.y(), relative_t.z(),
                     relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                     relative_yaw;
    loop_info = connected_info;
}

Eigen::Vector3d KeyFrame::getLoopRelativeT()
{
    assert(update_loop_info == 1);
    unique_lock<mutex> lock(mLoopInfo);
    return Eigen::Vector3d(loop_info(0), loop_info(1), loop_info(2));
}

double KeyFrame::getLoopRelativeYaw()
{
    assert(update_loop_info == 1);
    unique_lock<mutex> lock(mLoopInfo);
    return loop_info(7);
}

void KeyFrame::detectLoop(int index)
{
    has_loop = true;
    loop_index = index;
}

void KeyFrame::removeLoop()
{
    has_loop = false;
    update_loop_info = 0;
}

/**
 * @brief 汉明距离:对两个字符串进行异或运算，并统计结果为1的个数
 * @param a
 * @param b
 * @return
 */
int KeyFrame::HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b)
{
    BRIEF::bitset xor_of_bitset = a ^ b;

    // count()函数统计容器中所有值为1的元素的数量
    int dis = xor_of_bitset.count();
    return dis;
}

BriefExtractor::BriefExtractor(const std::string &pattern_file)
{
  // The DVision::BRIEF extractor computes a random pattern by default when
  // the object is created.
  // We load the pattern that we used to build the vocabulary, to make
  // the descriptors compatible with the predefined vocabulary
  
  // loads the pattern
  cv::FileStorage fs(pattern_file.c_str(), cv::FileStorage::READ);
  if(!fs.isOpened()) throw string("Could not open file ") + pattern_file;
  
  vector<int> x1, y1, x2, y2;
  fs["x1"] >> x1;
  fs["x2"] >> x2;
  fs["y1"] >> y1;
  fs["y2"] >> y2;
  
  m_brief.importPairs(x1, y1, x2, y2);
}

/**
 * @brief 计算图像帧中的特征点和描述子
 * @param im 图像帧
 * @param window_pts 2D图像特征点
 * @param keys 输出特征点
 * @param descriptors 输出描述子
 */
void BriefExtractor::operator() (const cv::Mat &im, const std::vector<cv::Point2f> window_pts,
                                 vector<cv::KeyPoint> &keys, vector<BRIEF::bitset> &descriptors) const
{
  // extract FAST keypoints with opencv
  const int fast_th = 20; // corner detector response threshold

  // im:为输入灰度图像;keys:为检测到的特征点向量;
  // fast_th:为阈值t; true:表示进行非极大值抑制
  cv::FAST(im, keys, fast_th, true);

  // 除了新提取的特征点意外还需要添加window_pts的特征
  // 添加之前在视觉追踪前端提取的特征点window_pts，是在KeyFrame::buildKeyFrameFeatures()中提取的
  for(int i = 0; i < (int)window_pts.size(); i++)
  {
      cv::KeyPoint key;

      // pt：关键点坐标；
      key.pt = window_pts[i];
      keys.push_back(key);
  }
  // compute their BRIEF descriptor
  // 通过ThirdParty来计算BRIEF描述子
  // im：输入图像；keys：输入特征点；descriptors：输出描述子；
  m_brief.compute(im, keys, descriptors);
}