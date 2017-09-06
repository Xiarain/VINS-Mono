#include "pose_local_parameterization.h"

/**
 * Ceres局部参数化，详细见官网
 */
/**
 * @brief 两个相机位置相加，包括旋转量和位移量
 * @param x 第一个相机位置
 * @param delta 第二个相机位置
 * @param x_plus_delta 输出两个相机位置之和
 * @return
 */
bool PoseLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    // Eigen::Map 将c++中普通格式的数据转换为Eigen中的矩阵格式
    // x，delta为三个位移量和一个四元数
    Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

    Eigen::Map<const Eigen::Vector3d> dp(delta);

    Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    p = _p + dp;
    q = (_q * dq).normalized();

    return true;
}
/**
 * @brief 初始化雅克比矩阵，
 * @param x
 * @param jacobian 输出雅克比矩阵
 * @return
 */
bool PoseLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);

    // 雅克比矩阵的前6行为单位阵
    j.topRows<6>().setIdentity();

    // 雅克比矩阵的最后一行为0
    j.bottomRows<1>().setZero();

    return true;
}
