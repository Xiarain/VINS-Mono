#include "camodocal/calib/CameraCalibration.h"

#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/sparse_graph/Transform.h"
#include "camodocal/gpl/EigenQuaternionParameterization.h"
#include "camodocal/gpl/EigenUtils.h"
#include "camodocal/camera_models/CostFunctionFactory.h"

#include "ceres/ceres.h"
namespace camodocal
{

CameraCalibration::CameraCalibration()
 : m_boardSize(cv::Size(0,0))
 , m_squareSize(0.0f)
 , m_verbose(false)
{

}

CameraCalibration::CameraCalibration(const Camera::ModelType modelType,
                                     const std::string& cameraName,
                                     const cv::Size& imageSize,
                                     const cv::Size& boardSize,
                                     float squareSize)
 : m_boardSize(boardSize)
 , m_squareSize(squareSize)
 , m_verbose(false)
{
    // 根据相机模型的种类，相机的名字，图片的大小，构造一个相机变量
    m_camera = CameraFactory::instance()->generateCamera(modelType, cameraName, imageSize);
}

void
CameraCalibration::clear(void)
{
    m_imagePoints.clear();
    m_scenePoints.clear();
}

/**
 * @brief 添加棋盘格数据，包括图像坐标系中的2D点，世界坐标系中的3D点
 * @param corners 角点图像坐标
 */
void
CameraCalibration::addChessboardData(const std::vector<cv::Point2f>& corners)
{
    // 2D点就是图像中检测到角点坐标，3D点就是按第一个为世界坐标系的原点，其他按棋盘格的实际尺寸计算
    // 可以理解为棋盘格静止，保持不动，所以3D点位置中的Z轴方向数据为0，棋格盘运动可以理解为就是相机在运动
    m_imagePoints.push_back(corners);

    std::vector<cv::Point3f> scenePointsInView;
    for (int i = 0; i < m_boardSize.height; ++i)
    {
        for (int j = 0; j < m_boardSize.width; ++j)
        {
            scenePointsInView.push_back(cv::Point3f(i * m_squareSize, j * m_squareSize, 0.0));
        }
    }
    m_scenePoints.push_back(scenePointsInView);
}

bool
CameraCalibration::calibrate(void)
{
    int imageCount = m_imagePoints.size();

    // compute intrinsic camera parameters and extrinsic parameters for each of the views
    // 计算每一帧的相机内参和外参
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    bool ret = calibrateHelper(m_camera, rvecs, tvecs);

    m_cameraPoses = cv::Mat(imageCount, 6, CV_64F);
    for (int i = 0; i < imageCount; ++i)
    {
        m_cameraPoses.at<double>(i,0) = rvecs.at(i).at<double>(0);
        m_cameraPoses.at<double>(i,1) = rvecs.at(i).at<double>(1);
        m_cameraPoses.at<double>(i,2) = rvecs.at(i).at<double>(2);
        m_cameraPoses.at<double>(i,3) = tvecs.at(i).at<double>(0);
        m_cameraPoses.at<double>(i,4) = tvecs.at(i).at<double>(1);
        m_cameraPoses.at<double>(i,5) = tvecs.at(i).at<double>(2);
    }

    // Compute measurement covariance.
    // 计算相机测量协方差
    std::vector<std::vector<cv::Point2f> > errVec(m_imagePoints.size());
    Eigen::Vector2d errSum = Eigen::Vector2d::Zero();
    size_t errCount = 0;
    for (size_t i = 0; i < m_imagePoints.size(); ++i)
    {
        std::vector<cv::Point2f> estImagePoints;

        // 3D点投影到2D点
        m_camera->projectPoints(m_scenePoints.at(i), rvecs.at(i), tvecs.at(i),
                                estImagePoints);

        for (size_t j = 0; j < m_imagePoints.at(i).size(); ++j)
        {
            // 观测值
            cv::Point2f pObs = m_imagePoints.at(i).at(j);

            // 估计值
            cv::Point2f pEst = estImagePoints.at(j);

            cv::Point2f err = pObs - pEst;

            errVec.at(i).push_back(err);

            errSum += Eigen::Vector2d(err.x, err.y);
        }

        errCount += m_imagePoints.at(i).size();
    }

    // 平均值
    Eigen::Vector2d errMean = errSum / static_cast<double>(errCount);

    Eigen::Matrix2d measurementCovariance = Eigen::Matrix2d::Zero();
    for (size_t i = 0; i < errVec.size(); ++i)
    {
        for (size_t j = 0; j < errVec.at(i).size(); ++j)
        {
            cv::Point2f err = errVec.at(i).at(j);
            double d0 = err.x - errMean(0);
            double d1 = err.y - errMean(1);

            measurementCovariance(0,0) += d0 * d0;
            measurementCovariance(0,1) += d0 * d1;
            measurementCovariance(1,1) += d1 * d1;
        }
    }
    measurementCovariance /= static_cast<double>(errCount);
    measurementCovariance(1,0) = measurementCovariance(0,1);

    m_measurementCovariance = measurementCovariance;

    return ret;
}

int
CameraCalibration::sampleCount(void) const
{
    return m_imagePoints.size();
}

std::vector<std::vector<cv::Point2f> >&
CameraCalibration::imagePoints(void)
{
    return m_imagePoints;
}

const std::vector<std::vector<cv::Point2f> >&
CameraCalibration::imagePoints(void) const
{
    return m_imagePoints;
}

std::vector<std::vector<cv::Point3f> >&
CameraCalibration::scenePoints(void)
{
    return m_scenePoints;
}

const std::vector<std::vector<cv::Point3f> >&
CameraCalibration::scenePoints(void) const
{
    return m_scenePoints;
}

CameraPtr&
CameraCalibration::camera(void)
{
    return m_camera;
}

const CameraConstPtr
CameraCalibration::camera(void) const
{
    return m_camera;
}

Eigen::Matrix2d&
CameraCalibration::measurementCovariance(void)
{
    return m_measurementCovariance;
}

const Eigen::Matrix2d&
CameraCalibration::measurementCovariance(void) const
{
    return m_measurementCovariance;
}

cv::Mat&
CameraCalibration::cameraPoses(void)
{
    return m_cameraPoses;
}

const cv::Mat&
CameraCalibration::cameraPoses(void) const
{
    return m_cameraPoses;
}

void
CameraCalibration::drawResults(std::vector<cv::Mat>& images) const
{
    std::vector<cv::Mat> rvecs, tvecs;

    for (size_t i = 0; i < images.size(); ++i)
    {
        cv::Mat rvec(3, 1, CV_64F);
        rvec.at<double>(0) = m_cameraPoses.at<double>(i,0);
        rvec.at<double>(1) = m_cameraPoses.at<double>(i,1);
        rvec.at<double>(2) = m_cameraPoses.at<double>(i,2);

        cv::Mat tvec(3, 1, CV_64F);
        tvec.at<double>(0) = m_cameraPoses.at<double>(i,3);
        tvec.at<double>(1) = m_cameraPoses.at<double>(i,4);
        tvec.at<double>(2) = m_cameraPoses.at<double>(i,5);

        rvecs.push_back(rvec);
        tvecs.push_back(tvec);
    }

    int drawShiftBits = 4;
    int drawMultiplier = 1 << drawShiftBits;

    cv::Scalar green(0, 255, 0);
    cv::Scalar red(0, 0, 255);

    for (size_t i = 0; i < images.size(); ++i)
    {
        cv::Mat& image = images.at(i);
        if (image.channels() == 1)
        {
            cv::cvtColor(image, image, CV_GRAY2RGB);
        }

        std::vector<cv::Point2f> estImagePoints;
        m_camera->projectPoints(m_scenePoints.at(i), rvecs.at(i), tvecs.at(i),
                                estImagePoints);

        float errorSum = 0.0f;
        float errorMax = std::numeric_limits<float>::min();

        for (size_t j = 0; j < m_imagePoints.at(i).size(); ++j)
        {
            cv::Point2f pObs = m_imagePoints.at(i).at(j);
            cv::Point2f pEst = estImagePoints.at(j);

            cv::circle(image,
                       cv::Point(cvRound(pObs.x * drawMultiplier),
                                 cvRound(pObs.y * drawMultiplier)),
                       5, green, 2, CV_AA, drawShiftBits);

            cv::circle(image,
                       cv::Point(cvRound(pEst.x * drawMultiplier),
                                 cvRound(pEst.y * drawMultiplier)),
                       5, red, 2, CV_AA, drawShiftBits);

            float error = cv::norm(pObs - pEst);

            errorSum += error;
            if (error > errorMax)
            {
                errorMax = error;
            }
        }

        std::ostringstream oss;
        oss << "Reprojection error: avg = " << errorSum / m_imagePoints.at(i).size()
            << "   max = " << errorMax;

        cv::putText(image, oss.str(), cv::Point(10, image.rows - 10),
                    cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255),
                    1, CV_AA);
    }
}

void
CameraCalibration::writeParams(const std::string& filename) const
{
    m_camera->writeParametersToYamlFile(filename);
}

bool
CameraCalibration::writeChessboardData(const std::string& filename) const
{
    std::ofstream ofs(filename.c_str(), std::ios::out | std::ios::binary);
    if (!ofs.is_open())
    {
        return false;
    }

    writeData(ofs, m_boardSize.width);
    writeData(ofs, m_boardSize.height);
    writeData(ofs, m_squareSize);

    writeData(ofs, m_measurementCovariance(0,0));
    writeData(ofs, m_measurementCovariance(0,1));
    writeData(ofs, m_measurementCovariance(1,0));
    writeData(ofs, m_measurementCovariance(1,1));

    writeData(ofs, m_cameraPoses.rows);
    writeData(ofs, m_cameraPoses.cols);
    writeData(ofs, m_cameraPoses.type());
    for (int i = 0; i < m_cameraPoses.rows; ++i)
    {
        for (int j = 0; j < m_cameraPoses.cols; ++j)
        {
            writeData(ofs, m_cameraPoses.at<double>(i,j));
        }
    }

    writeData(ofs, m_imagePoints.size());
    for (size_t i = 0; i < m_imagePoints.size(); ++i)
    {
        writeData(ofs, m_imagePoints.at(i).size());
        for (size_t j = 0; j < m_imagePoints.at(i).size(); ++j)
        {
            const cv::Point2f& ipt = m_imagePoints.at(i).at(j);

            writeData(ofs, ipt.x);
            writeData(ofs, ipt.y);
        }
    }

    writeData(ofs, m_scenePoints.size());
    for (size_t i = 0; i < m_scenePoints.size(); ++i)
    {
        writeData(ofs, m_scenePoints.at(i).size());
        for (size_t j = 0; j < m_scenePoints.at(i).size(); ++j)
        {
            const cv::Point3f& spt = m_scenePoints.at(i).at(j);

            writeData(ofs, spt.x);
            writeData(ofs, spt.y);
            writeData(ofs, spt.z);
        }
    }

    return true;
}

bool
CameraCalibration::readChessboardData(const std::string& filename)
{
    std::ifstream ifs(filename.c_str(), std::ios::in | std::ios::binary);
    if (!ifs.is_open())
    {
        return false;
    }

    readData(ifs, m_boardSize.width);
    readData(ifs, m_boardSize.height);
    readData(ifs, m_squareSize);

    readData(ifs, m_measurementCovariance(0,0));
    readData(ifs, m_measurementCovariance(0,1));
    readData(ifs, m_measurementCovariance(1,0));
    readData(ifs, m_measurementCovariance(1,1));

    int rows, cols, type;
    readData(ifs, rows);
    readData(ifs, cols);
    readData(ifs, type);
    m_cameraPoses = cv::Mat(rows, cols, type);

    for (int i = 0; i < m_cameraPoses.rows; ++i)
    {
        for (int j = 0; j < m_cameraPoses.cols; ++j)
        {
            readData(ifs, m_cameraPoses.at<double>(i,j));
        }
    }

    size_t nImagePointSets;
    readData(ifs, nImagePointSets);

    m_imagePoints.clear();
    m_imagePoints.resize(nImagePointSets);
    for (size_t i = 0; i < m_imagePoints.size(); ++i)
    {
        size_t nImagePoints;
        readData(ifs, nImagePoints);
        m_imagePoints.at(i).resize(nImagePoints);

        for (size_t j = 0; j < m_imagePoints.at(i).size(); ++j)
        {
            cv::Point2f& ipt = m_imagePoints.at(i).at(j);
            readData(ifs, ipt.x);
            readData(ifs, ipt.y);
        }
    }

    size_t nScenePointSets;
    readData(ifs, nScenePointSets);

    m_scenePoints.clear();
    m_scenePoints.resize(nScenePointSets);
    for (size_t i = 0; i < m_scenePoints.size(); ++i)
    {
        size_t nScenePoints;
        readData(ifs, nScenePoints);
        m_scenePoints.at(i).resize(nScenePoints);

        for (size_t j = 0; j < m_scenePoints.at(i).size(); ++j)
        {
            cv::Point3f& spt = m_scenePoints.at(i).at(j);
            readData(ifs, spt.x);
            readData(ifs, spt.y);
            readData(ifs, spt.z);
        }
    }

    return true;
}

void
CameraCalibration::setVerbose(bool verbose)
{
    m_verbose = verbose;
}

bool
CameraCalibration::calibrateHelper(CameraPtr& camera,
                                   std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) const
{
    rvecs.assign(m_scenePoints.size(), cv::Mat());
    tvecs.assign(m_scenePoints.size(), cv::Mat());

    // STEP 1: Estimate intrinsics
    // SETP 1： 估计相机内参
    camera->estimateIntrinsics(m_boardSize, m_scenePoints, m_imagePoints);

    // STEP 2: Estimate extrinsics
    // STEP 2： 估计相机外参
    for (size_t i = 0; i < m_scenePoints.size(); ++i)
    {
        // 世界坐标系下的3D点，图像坐标系的2D点，求解的旋转量，求解的平移量
        camera->estimateExtrinsics(m_scenePoints.at(i), m_imagePoints.at(i), rvecs.at(i), tvecs.at(i));
    }

    if (m_verbose)
    {
        std::cout << "[" << camera->cameraName() << "] "
                  << "# INFO: " << "Initial reprojection error: "
                  << std::fixed << std::setprecision(3)
                  << camera->reprojectionError(m_scenePoints, m_imagePoints, rvecs, tvecs)
                  << " pixels" << std::endl;
    }

    // STEP 3: optimization using ceres
    // 将上面求解的相机内参，相机外参，通过ceres优化重投影误差
    optimize(camera, rvecs, tvecs);

    if (m_verbose)
    {
        double err = camera->reprojectionError(m_scenePoints, m_imagePoints, rvecs, tvecs);
        std::cout << "[" << camera->cameraName() << "] " << "# INFO: Final reprojection error: "
                  << err << " pixels" << std::endl;
        std::cout << "[" << camera->cameraName() << "] " << "# INFO: "
                  << camera->parametersToString() << std::endl;
    }

    return true;
}

/**
 * @brief 优化
 * @param camera 相机指针
 * @param rvecs PnP求解出来的旋转结果
 * @param tvecs PnP求解出来的平移结果
 */
void
CameraCalibration::optimize(CameraPtr& camera,
                            std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) const
{
    // Use ceres to do optimization
    ceres::Problem problem;

    // 相机位置Transform类 R{q},t
    std::vector<Transform, Eigen::aligned_allocator<Transform> > transformVec(rvecs.size());
    for (size_t i = 0; i < rvecs.size(); ++i)
    {
        Eigen::Vector3d rvec;
        cv::cv2eigen(rvecs.at(i), rvec);

        transformVec.at(i).rotation() = Eigen::AngleAxisd(rvec.norm(), rvec.normalized());
        transformVec.at(i).translation() << tvecs[i].at<double>(0),
                                            tvecs[i].at<double>(1),
                                            tvecs[i].at<double>(2);
    }

    std::vector<double> intrinsicCameraParams;
    m_camera->writeParameters(intrinsicCameraParams);

    // create residuals for each observation
    // 创建每一个观测值的残差
    for (size_t i = 0; i < m_imagePoints.size(); ++i)
    {
        for (size_t j = 0; j < m_imagePoints.at(i).size(); ++j)
        {
            const cv::Point3f& spt = m_scenePoints.at(i).at(j);
            const cv::Point2f& ipt = m_imagePoints.at(i).at(j);

            // 在CostFuntionFactory.cc 中定义了七种代价函数（generateCostFunction()），这个些不同的代价函数考虑了不同的优化对象，如相机位置、
            // 2D3D特征点的位置、二视图、平方根精度等
            // 向代价函数添加3D和2D观测值
            ceres::CostFunction* costFunction =
                CostFunctionFactory::instance()->generateCostFunction(camera,
                                                                      Eigen::Vector3d(spt.x, spt.y, spt.z),
                                                                      Eigen::Vector2d(ipt.x, ipt.y),
                                                                      CAMERA_INTRINSICS | CAMERA_POSE);

            // 损失函数
            // 核函数，用来减小Outlier的影响
            ceres::LossFunction* lossFunction = new ceres::CauchyLoss(1.0);

            // 设置残差
            // 在这个函数中设置的相机内参、相机旋转量、相机平移量是优化过程中优化的变量
            problem.AddResidualBlock(costFunction, lossFunction,
                                     intrinsicCameraParams.data(),
                                     transformVec.at(i).rotationData(),
                                     transformVec.at(i).translationData());
        }

        // LocalParameterization 对于每一个参数块只是在参数块的局部切空间产生扰动
        ceres::LocalParameterization* quaternionParameterization =
            new EigenQuaternionParameterization;

        // 这个参数块设置一次后就不能被改变了
        problem.SetParameterization(transformVec.at(i).rotationData(),
                                    quaternionParameterization);
    }

    std::cout << "begin ceres" << std::endl;
    ceres::Solver::Options options;
    options.max_num_iterations = 1000;
    options.num_threads = 1;

    if (m_verbose)
    {
        options.minimizer_progress_to_stdout = true;
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << "end ceres" << std::endl;

    // 是否输出ceres优化结果报告
    if (m_verbose)
    {
        std::cout << summary.FullReport() << std::endl;
    }

    camera->readParameters(intrinsicCameraParams);

    for (size_t i = 0; i < rvecs.size(); ++i)
    {
        Eigen::AngleAxisd aa(transformVec.at(i).rotation());

        Eigen::Vector3d rvec = aa.angle() * aa.axis();
        cv::eigen2cv(rvec, rvecs.at(i));

        cv::Mat& tvec = tvecs.at(i);
        tvec.at<double>(0) = transformVec.at(i).translation()(0);
        tvec.at<double>(1) = transformVec.at(i).translation()(1);
        tvec.at<double>(2) = transformVec.at(i).translation()(2);
    }
}

template<typename T>
void
CameraCalibration::readData(std::ifstream& ifs, T& data) const
{
    char* buffer = new char[sizeof(T)];

    ifs.read(buffer, sizeof(T));

    data = *(reinterpret_cast<T*>(buffer));

    delete[] buffer;
}

template<typename T>
void
CameraCalibration::writeData(std::ofstream& ofs, T data) const
{
    char* pData = reinterpret_cast<char*>(&data);

    ofs.write(pData, sizeof(T));
}

}
