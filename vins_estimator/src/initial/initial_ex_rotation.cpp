#include "initial_ex_rotation.h"

InitialEXRotation::InitialEXRotation(){
    frame_count = 0;
    Rc.push_back(Matrix3d::Identity());
    Rc_g.push_back(Matrix3d::Identity());
    Rimu.push_back(Matrix3d::Identity());
    ric = Matrix3d::Identity();
}

/**
 * @brief  通过对齐camera和IMU旋转的序列中求得camera和IMU旋转偏移常量
 * @param corres 两幅图像中的匹配角点
 * @param delta_q_imu IMU预积分值
 * @param calib_ric_result SVD分解奇异值向量
 * @return 是否成功得到外部旋转量
 */
bool InitialEXRotation::CalibrationExRotation(vector<pair<Vector3d, Vector3d>> corres, Quaterniond delta_q_imu, Matrix3d &calib_ric_result)
{
    // Moncular Visual-Inertial State Estimation With Online Initializaiton 公式（5）
    frame_count++;

    // 通过图像匹配角点获得两个图像之间的旋转量
    Rc.push_back(solveRelativeR(corres));

    // 通过IMU预计分获得旋转量
    Rimu.push_back(delta_q_imu.toRotationMatrix());

    // Moncular Visual-Inertial State Estimation With Online Initializaiton 公式（8）
    // {\hat R_{c}^{b}}^{-1}  R_{b_{k+1}}^{b_{k}}  \hat R_{c}^{b}
    // delta_q_imu 是从机体坐标k帧到机体坐标k+1帧的旋转量
    // ric 是从相机坐标系到机体坐标系的旋转量
    Rc_g.push_back(ric.inverse() * delta_q_imu * ric);

    Eigen::MatrixXd A(frame_count * 4, 4);
    A.setZero();
    int sum_ok = 0;
    for (int i = 1; i <= frame_count; i++)
    {
        Quaterniond r1(Rc[i]);
        Quaterniond r2(Rc_g[i]);

        // 求解两个旋转之间的夹角
        // d = q1 \otimes q2*;
        // theta = atan2(d.vec().norm(), d.w());
        double angular_distance = 180 / M_PI * r1.angularDistance(r2);
        ROS_DEBUG(
            "%d %f", i, angular_distance);

        // Moncular Visual-Inertial State Estimation With Online Initializaiton 公式（5）
        // 权值的计算
        double huber = angular_distance > 5.0 ? 5.0 / angular_distance : 1.0;
        ++sum_ok;
        Matrix4d L, R;

        // camera 旋转量
        double w = Quaterniond(Rc[i]).w();
        Vector3d q = Quaterniond(Rc[i]).vec();
        L.block<3, 3>(0, 0) = w * Matrix3d::Identity() + Utility::skewSymmetric(q);
        L.block<3, 1>(0, 3) = q;
        L.block<1, 3>(3, 0) = -q.transpose();
        L(3, 3) = w;

        // IMU 旋转量
        Quaterniond R_ij(Rimu[i]);
        w = R_ij.w();
        q = R_ij.vec();
        R.block<3, 3>(0, 0) = w * Matrix3d::Identity() - Utility::skewSymmetric(q);
        R.block<3, 1>(0, 3) = q;
        R.block<1, 3>(3, 0) = -q.transpose();
        R(3, 3) = w;

        A.block<4, 4>((i - 1) * 4, 0) = huber * (L - R);
    }

    // Eigen 矩形矩阵的双边Jacobi SVD分解
    // 利用SVD求解Ax=0方程的解
    // A = USV* A为 m×n矩阵， n为4，所以最优解为V矩阵的第4列
    JacobiSVD<MatrixXd> svd(A, ComputeFullU | ComputeFullV);
    Matrix<double, 4, 1> x = svd.matrixV().col(3);

    // 由SVD分解的V矩阵的第三列构造旋转矩阵
    Quaterniond estimated_R(x);
    ric = estimated_R.toRotationMatrix().inverse();
    //cout << svd.singularValues().transpose() << endl;
    //cout << ric << endl;

    // 判断第二小的奇异值是否足够大，大于阀值
    // riv_cov SVD分解的奇异值
    Vector3d ric_cov;
    ric_cov = svd.singularValues().tail<3>();
    if (frame_count >= WINDOW_SIZE && ric_cov(1) > 0.25)
    {
        calib_ric_result = ric;
        return true;
    }
    else
        return false;
}

/**
 * @brief  计算两个图像之间的旋转量
 * @param corres 匹配的角点
 * @return
 */
Matrix3d InitialEXRotation::solveRelativeR(const vector<pair<Vector3d, Vector3d>> &corres)
{
    // 匹配的角点数要大于9对
    if (corres.size() >= 9)
    {
        vector<cv::Point2f> ll, rr;
        for (int i = 0; i < int(corres.size()); i++)
        {
            ll.push_back(cv::Point2f(corres[i].first(0), corres[i].first(1)));
            rr.push_back(cv::Point2f(corres[i].second(0), corres[i].second(1)));
        }

        // 用Opencv求解E矩阵
        cv::Mat E = cv::findFundamentalMat(ll, rr);
        cv::Mat_<double> R1, R2, t1, t2;
        decomposeE(E, R1, R2, t1, t2);

        // 判断行列式的值
        if (determinant(R1) + 1.0 < 1e-09)
        {
            E = -E;
            decomposeE(E, R1, R2, t1, t2);
        }
        double ratio1 = max(testTriangulation(ll, rr, R1, t1), testTriangulation(ll, rr, R1, t2));
        double ratio2 = max(testTriangulation(ll, rr, R2, t1), testTriangulation(ll, rr, R2, t2));
        cv::Mat_<double> ans_R_cv = ratio1 > ratio2 ? R1 : R2;

        // Eigen与opencv存储矩阵的格式是不一样的，Eigen是竖向，opencv是横向的（平时使用的矩阵形式）
        Matrix3d ans_R_eigen;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                ans_R_eigen(j, i) = ans_R_cv(i, j);
        return ans_R_eigen;
    }
    return Matrix3d::Identity();
}

/**
 * @brief  通过三角化来测试确定E矩阵分解
 * @param l  第一幅图像中特征点
 * @param r  第二幅图像中的特征点
 * @param R  旋转矩阵
 * @param t  平移量
 * @return 返回正向点云的比值
 */
double InitialEXRotation::testTriangulation(const vector<cv::Point2f> &l,
                                          const vector<cv::Point2f> &r,
                                          cv::Mat_<double> R, cv::Mat_<double> t)
{
    cv::Mat pointcloud;
    cv::Matx34f P = cv::Matx34f(1, 0, 0, 0,
                                0, 1, 0, 0,
                                0, 0, 1, 0);
    cv::Matx34f P1 = cv::Matx34f(R(0, 0), R(0, 1), R(0, 2), t(0),
                                 R(1, 0), R(1, 1), R(1, 2), t(1),
                                 R(2, 0), R(2, 1), R(2, 2), t(2));
    // opencv 通过三角花重构2D点
    // 第一个相机投影矩阵 第二个相机投影矩阵 第一幅图像中的特征点 第二幅图像中的特征点 输出在齐次坐标系下的3D点云
    cv::triangulatePoints(P, P1, l, r, pointcloud);

    int front_count = 0;
    for (int i = 0; i < pointcloud.cols; i++)
    {
        // 将齐次坐标系下的3D点云转换为非齐次坐标系下的3D点云
        double normal_factor = pointcloud.col(i).at<float>(3);

        // 检测该点在两个相机下的深度，只有深度为正才是才是正确的解
        cv::Mat_<double> p_3d_l = cv::Mat(P) * (pointcloud.col(i) / normal_factor);
        cv::Mat_<double> p_3d_r = cv::Mat(P1) * (pointcloud.col(i) / normal_factor);
        if (p_3d_l(2) > 0 && p_3d_r(2) > 0)
            front_count++;
    }
    ROS_DEBUG("MotionEstimator: %f", 1.0 * front_count / pointcloud.cols);
    return 1.0 * front_count / pointcloud.cols;
}

/**
 * @brief E矩阵分解，详细见多视觉几何 P175 结论8.19
 * @param E E本质矩阵
 * @param R1 返回旋转矩阵R1
 * @param R2 返回旋转矩阵R2
 * @param t1 返回平移量t1
 * @param t2 返回平移量t2
 */
void InitialEXRotation::decomposeE(cv::Mat E,
                                 cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                                 cv::Mat_<double> &t1, cv::Mat_<double> &t2)
{
    cv::SVD svd(E, cv::SVD::MODIFY_A);
    cv::Matx33d W(0, -1, 0,
                  1, 0, 0,
                  0, 0, 1);
    cv::Matx33d Wt(0, 1, 0,
                   -1, 0, 0,
                   0, 0, 1);
    R1 = svd.u * cv::Mat(W) * svd.vt;
    R2 = svd.u * cv::Mat(Wt) * svd.vt;
    t1 = svd.u.col(2);
    t2 = -svd.u.col(2);
}
