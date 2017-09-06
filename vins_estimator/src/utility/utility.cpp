#include "utility.h"

/**
 * @brief 通过将实际重力加速度旋转理想重力加速度得到旋转矩阵
 * @param g 实际重力加速度
 * @return
 */
Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d &g)
{
    Eigen::Matrix3d R0;
    Eigen::Vector3d ng1 = g.normalized();
    Eigen::Vector3d ng2{0, 0, 1.0};

    // 所建立的旋转代表一个旋转，将方向ng1发送到方向ng2线，这两条线通过原点。
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
    return R0;
}
