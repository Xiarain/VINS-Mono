#include "feature_tracker.h"

int FeatureTracker::n_id = 0;

bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

/**
 * @brief 去除状态不好的状态量
 * @param v 2D特征点
 * @param status 状态
 */
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];

    // 根据状态好的变量重新配置大小
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

FeatureTracker::FeatureTracker()
{
}

/**
 * @brief 在VINS-Mono/config有一张fisheye_mask.jpg图片
 *        在配置的时候会读取这张图片
 */
void FeatureTracker::setMask()
{
    // 如果是鱼眼相机的话，之前读取的配置文件夹中的鱼眼模型图像就会被使用，
    // 但是如果只是普通的相机的话，就是一个白色等大的图像，也就是没有舍弃特征点
    if(FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    

    // prefer to keep features that are tracked for long time
    // 选择保留那些跟踪时间更长的特征点
    // cnt_pts_id 数据结构： int cv::Point2f int
    //            对应实际数据结构： 应该是被跟踪到的次数 输出被找到的2D点 ID号
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

    // 将特征点通过被追踪的次数进行排序，排序的一句就是特征点被追踪的次数
    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    // 重置被找到的2D点
    forw_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        // 查看这个区域是否是白色
        if (mask.at<uchar>(it.second.first) == 255)
        {
            // 在Mask白色区域内，则将之前排序的cnt_pts_id数据结构填入相应的向量中
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);

            // mask：图像指针 it.second.first：画圆的中心点 MIN_DIST：圆的半径 0：圆的颜色 -1：画圆线条的粗细
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);

        // 被检测到的次数为一次
        track_cnt.push_back(1);
    }
}

void FeatureTracker::readImage(const cv::Mat &_img)
{
    cv::Mat img;
    TicToc t_r;

    // 直方图均衡
    if (EQUALIZE)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;

        // 进行直方图均衡，
        // _img 原图像
        // img 均衡后的结果图像
        clahe->apply(_img, img);
        ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
        img = _img;

    if (forw_img.empty())
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }

    forw_pts.clear();

    // 上一幅图像中的2D特征点已经找到了
    if (cur_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;

        // 金字塔光流法
        // cur_img 当前图像 forw_img 第二幅图像
        // cur_pts 需要被找到2D点 forw_pts 输出被找到的2D点
        // status 光流法对应每一个特征点跟踪的结果
        // err 对应每一个特征点的误差
        // cv::Size(21, 21) 每一层金字塔搜索窗口的大小
        // 3 金字塔层数
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

        for (int i = 0; i < int(forw_pts.size()); i++)
            if (status[i] && !inBorder(forw_pts[i]))
                status[i] = 0;
        // 前一帧追踪2D点
        reduceVector(prev_pts, status);

        // 当前帧追踪2D点
        reduceVector(cur_pts, status);

        // 下一帧追踪2D点
        reduceVector(forw_pts, status);

        // 特征点ID号
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
    }

    // 只要相机的图像帧时间戳没有问题，这个帧就会被发送
    if (PUB_THIS_FRAME)
    {
        // 通过F矩阵去除一些外点
        rejectWithF();

        // 将检测到的特征点被追踪到的次数加一
        for (auto &n : track_cnt)
            n++;

        ROS_DEBUG("set mask begins");
        TicToc t_m;

        // 设置遮挡部分
        setMask();
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != forw_img.size())
                cout << "wrong size " << endl;

            // 第一帧初始化特征点检测也是通过这里完成的

            // 确定图像中的强角点
            // forw_img：输入图像
            // n_pts：输出检测到的角点
            // MAX_CNT - forw_pts.size()：被查找的最大角点数
            // 0.1： 最小能被接受的角点质量
            // MIN_DIST： 被查找到角点最小的欧式距离
            // mask：感兴趣的区域
            // 指示是否使用Harris角点检测，如不指定，则计算shi-tomasi角点
            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.1, MIN_DIST, mask);
        }
        else
            n_pts.clear();
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        ROS_DEBUG("add feature begins");
        TicToc t_a;

        // 把上面新检测到特征点(n_pts)添加到特征点容器(forw_pts)中
        addPoints();
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());

        prev_img = forw_img;
        prev_pts = forw_pts;
    }
    cur_img = forw_img;
    cur_pts = forw_pts;
}

/**
 * @brief 通过F矩阵排除一些外点
 */
void FeatureTracker::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;

        // 前一帧追踪的2D点，下一帧追踪的2D点
        vector<cv::Point2f> un_prev_pts(prev_pts.size()), un_forw_pts(forw_pts.size());

        for (unsigned int i = 0; i < prev_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            // tmp_p 输出3D点
            // TODO 把检测到特征点加入相机畸变矫正模型转换到像素坐标系上？？？
            // 将一个点从图像坐标系转换到像素坐标系将图像坐标系
            m_camera->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y), tmp_p);

            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;

            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;

            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;

        // 输入第一幅图像中匹配的特征点 输入第二幅图像中匹配的特征点 计算基础矩阵的的方式（RANSAC N>8）
        // 设置RANSAC的参数：最大距离，设置RANSAC的参数：置信度
        // 输出要素，这些要素标记外点为0，内点为1
        cv::findFundamentalMat(un_prev_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = prev_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

/**
 * @brief 更新ID号，如果这个ID中数据为-1，则将目前的特征点计数赋值给这个ID号，同时特征点计数自增
 * @param i 特征点数量
 * @return
 */
bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

/**
 * @brief 读取相机内参参数
 * @param calib_file
 */
void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;

    // 将理想像素平面转换到一个标准平面，加入相机畸变模型
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }

    // 转换到像素平面
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }

    // 显示图片
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}

/**
 * @brief 将所有特征点转换到一个标准化平面并且进行畸变
 * @return
 */
vector<cv::Point2f> FeatureTracker::undistortedPoints()
{
    vector<cv::Point2f> un_pts;
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    }

    return un_pts;
}
