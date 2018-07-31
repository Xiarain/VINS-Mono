#include "initial_alignment.h"

/**
 * @brief IMU陀螺仪零偏初始化
 * @param all_image_frame 图像观测值与IMU预积分
 * @param Bgs 输出陀螺仪的零偏
 */
void solveGyroscopeBias(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs)
{
    Matrix3d A;
    Vector3d b;
    Vector3d delta_bg;
    A.setZero();
    b.setZero();
    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++)
    {
        frame_j = next(frame_i);
        MatrixXd tmp_A(3, 3);
        tmp_A.setZero();
        VectorXd tmp_b(3);
        tmp_b.setZero();

        // Technical Report VINS-Mono 公式（15）
        // 把公式（15）第二条公式带入第一条公式
        // 然后把[1 bg]这个四元数拆解分为[0 bg] + [1 0]
        // TODO 公式推到出来的跟实际代码对应不上，tmp_A 应该还需要乘以frame_j->second.pre_integration->delta_q.inverse() * q_ij
        Eigen::Quaterniond q_ij(frame_i->second.R.transpose() * frame_j->second.R);

        // O_R O_BG是矩阵的位置
        tmp_A = frame_j->second.pre_integration->jacobian.template block<3, 3>(O_R, O_BG);

        // Eigen::quanternion.vec() 返回四元数的虚部
        tmp_b = 2 * (frame_j->second.pre_integration->delta_q.inverse() * q_ij).vec();

        // 使用Eigen ldlt矩阵分解方法，需要将Ax=b这种形式转换为 A.T Ax=A.T b
        A += tmp_A.transpose() * tmp_A;
        b += tmp_A.transpose() * tmp_b;

    }

    // 稀疏矩阵分解，求解方程 Ax=b
    delta_bg = A.ldlt().solve(b);
    ROS_WARN_STREAM("gyroscope bias initial calibration " << delta_bg.transpose());

    for (int i = 0; i <= WINDOW_SIZE; i++)
        Bgs[i] += delta_bg;

    // 通过新求解的陀螺仪零偏更新预积分
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end( ); frame_i++)
    {
        // 是第frame_i帧的零偏给第frame_j帧的预积分
        frame_j = next(frame_i);
        frame_j->second.pre_integration->repropagate(Vector3d::Zero(), Bgs[0]);
    }
}

/**
 * @brief 计算重力加速度中的b1和b2
 * @param g0 初略估计的重力加速度
 * @return b1 b2 重力加速度分向量
 */
MatrixXd TangentBasis(Vector3d &g0)
{
    Vector3d b, c;
    Vector3d a = g0.normalized();
    Vector3d tmp(0, 0, 1);
    if(a == tmp)
        tmp << 1, 0, 0;

    // b1
    b = (tmp - a * (a.transpose() * tmp)).normalized();

    // b2 = g × b1
    c = a.cross(b);
    MatrixXd bc(3, 2);
    bc.block<3, 1>(0, 0) = b;
    bc.block<3, 1>(0, 1) = c;
    return bc;
}

/**
 * @brief 进一步细化重力加速度，提高估计值的精度
 * @param all_image_frame 图像观测值与IMU预积分
 * @param g 输入粗略估计的重力加速度，经过细化后输出
 * @param x 输出对齐后的速度、重力加速度、尺度因子
 */
void RefineGravity(map<double, ImageFrame> &all_image_frame, Vector3d &g, VectorXd &x)
{
    Vector3d g0 = g.normalized() * G.norm();
    Vector3d lx, ly;
    //VectorXd x;
    int all_frame_count = all_image_frame.size();
    int n_state = all_frame_count * 3 + 2 + 1;

    MatrixXd A{n_state, n_state};
    A.setZero();
    VectorXd b{n_state};
    b.setZero();

    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    for(int k = 0; k < 4; k++)
    {
        // lxly的形式:b1
        //           b2
        MatrixXd lxly(3, 2);
        lxly = TangentBasis(g0);
        int i = 0;
        for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
        {
            frame_j = next(frame_i);

            MatrixXd tmp_A(6, 9);
            tmp_A.setZero();
            VectorXd tmp_b(6);
            tmp_b.setZero();

            double dt = frame_j->second.pre_integration->sum_dt;

            // 这个公式与LinearAlignment（）函数的区别是，在Robust Initialization of Monocular Visual-Inertial 公式（7）
            // 用 g + w1b1 + w2b2来代替 g
            tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
            tmp_A.block<3, 2>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Matrix3d::Identity() * lxly;
            tmp_A.block<3, 1>(0, 8) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) / 100.0;     

            tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p + frame_i->second.R.transpose() * frame_j->second.R * TIC[0] - TIC[0] - frame_i->second.R.transpose() * dt * dt / 2 * g0;

            tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
            tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
            tmp_A.block<3, 2>(3, 6) = frame_i->second.R.transpose() * dt * Matrix3d::Identity() * lxly;

            tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v - frame_i->second.R.transpose() * dt * Matrix3d::Identity() * g0;


            Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
            //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
            //MatrixXd cov_inv = cov.inverse();
            cov_inv.setIdentity();

            MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
            VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

            A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
            b.segment<6>(i * 3) += r_b.head<6>();

            A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>();
            b.tail<3>() += r_b.tail<3>();

            A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
            A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();
        }
            A = A * 1000.0;
            b = b * 1000.0;
            x = A.ldlt().solve(b);

            // dg 重力在切空间上的两个分量
            VectorXd dg = x.segment<2>(n_state - 3);

            //
            g0 = (g0 + lxly * dg).normalized() * G.norm();
            //double s = x(n_state - 1);
    }   
    g = g0;
}

/**
 * @brief 速度，重力向量，尺度初始化
 * @param all_image_frame 图像观测值与IMU预积分
 * @param g 输出重力加速度
 * @param x 输出对齐后的速度、重力加速度、尺度因子
 * @return
 */
bool LinearAlignment(map<double, ImageFrame> &all_image_frame, Vector3d &g, VectorXd &x)
{
    // 所有的帧数
    int all_frame_count = all_image_frame.size();

    // Robust Initialization of Monocular Visual-Inertial Estimation on Aerial Robots 公式（6）
    // 定义所有的想要估计的变量
    // 帧数×3 + 重力加速度 + 尺度
    int n_state = all_frame_count * 3 + 3 + 1;

    // （帧数×3 + 重力加速度 + 尺度）×2
    // 位置预积分
    // 速度预积分
    MatrixXd A{n_state, n_state};
    A.setZero();


    VectorXd b{n_state};
    b.setZero();

    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    int i = 0;
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
    {
        frame_j = next(frame_i);

        MatrixXd tmp_A(6, 10);
        tmp_A.setZero();

        // IMU 预计分值
        // Robust Initialization of Monocular Visual-Inertial Estimation on Aerial Robots 公式（3）
        VectorXd tmp_b(6);
        tmp_b.setZero();

        // Technical Report VINS-Mone 公式（17）~(18)
        double dt = frame_j->second.pre_integration->sum_dt;

        // TODO tmp_A.block<3, 3>(0, 0)缺一个frame_i->second.R.transpose()
        tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
        tmp_A.block<3, 3>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Matrix3d::Identity();
        tmp_A.block<3, 1>(0, 9) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) / 100.0;     

        // TIC: camera和IMU外参的平移量，从外部参数列表获得
        // TODO 前部分是IMU预计分中的位置预计分，但是后面半部分是？
        tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p + frame_i->second.R.transpose() * frame_j->second.R * TIC[0] - TIC[0];
        //cout << "delta_p   " << frame_j->second.pre_integration->delta_p.transpose() << endl;

        tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
        tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
        tmp_A.block<3, 3>(3, 6) = frame_i->second.R.transpose() * dt * Matrix3d::Identity();

        // IMU 预积分的速度预积分
        tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v;
        //cout << "delta_v   " << frame_j->second.pre_integration->delta_v.transpose() << endl;

        Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
        //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
        //MatrixXd cov_inv = cov.inverse();
        cov_inv.setIdentity();

        // 对AX=b求解，利用最小二乘法，A.TZ = A.TAX
        // A.T*A矩阵是对称阵
        MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
        VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

        // A 矩阵 （all_frame_count * 3 + 3 + 1） × 2
        A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>(); // 左上6*6矩阵块
        b.segment<6>(i * 3) += r_b.head<6>(); // segment 从i+3位置的6个元素 head： 向量前6个元素

        A.bottomRightCorner<4, 4>() += r_A.bottomRightCorner<4, 4>(); // 右下4*4矩阵块
        b.tail<4>() += r_b.tail<4>(); //tail 向量后4个元素

        A.block<6, 4>(i * 3, n_state - 4) += r_A.topRightCorner<6, 4>(); // 右上6*4矩阵块
        A.block<4, 6>(n_state - 4, i * 3) += r_A.bottomLeftCorner<4, 6>(); //右下4*6矩阵块
    }
    A = A * 1000.0;
    b = b * 1000.0;

    // ldlt 矩阵分解
    x = A.ldlt().solve(b);
    double s = x(n_state - 1) / 100.0;
    ROS_DEBUG("estimated scale: %f", s);

    g = x.segment<3>(n_state - 4);
    ROS_DEBUG_STREAM(" result g     " << g.norm() << " " << g.transpose());

    if(fabs(g.norm() - G.norm()) > 1.0 || s < 0)
    {
        return false;
    }

    // 改进重力加速度的估计值
    RefineGravity(all_image_frame, g, x);

    s = (x.tail<1>())(0) / 100.0;
    (x.tail<1>())(0) = s;
    ROS_DEBUG_STREAM(" refine     " << g.norm() << " " << g.transpose());
    if(s < 0.0 )
        return false;   
    else
        return true;
}

/**
 * @brief camera与IMU对齐
 * @param all_image_frame 图像观测值与IMU预积分
 * @param Bgs 输出陀螺仪零偏
 * @param g 输出重力加速度
 * @param x 输出对齐后的速度、重力加速度、尺度因子
 * @return
 */
bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x)
{
    // IMU陀螺仪零偏初始化
    solveGyroscopeBias(all_image_frame, Bgs);

    // 速度，重力向量，尺度初始化
    if(LinearAlignment(all_image_frame, g, x))
        return true;
    else 
        return false;
}
