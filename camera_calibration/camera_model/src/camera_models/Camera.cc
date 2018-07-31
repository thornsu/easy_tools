#include "camodocal/camera_models/Camera.h"
#include "camodocal/camera_models/ScaramuzzaCamera.h"

#include <opencv2/calib3d/calib3d.hpp>

namespace camodocal
{
//! 初始化相机模型
Camera::Parameters::Parameters(ModelType modelType)
 : m_modelType(modelType)
 , m_imageWidth(0)
 , m_imageHeight(0)
{
    switch (modelType)
    {
    case KANNALA_BRANDT:
        m_nIntrinsics = 8;
        break;
    case PINHOLE:
        m_nIntrinsics = 8;
        break;
    case SCARAMUZZA:
        m_nIntrinsics = SCARAMUZZA_CAMERA_NUM_PARAMS;
        break;
    case MEI:
    default:
        m_nIntrinsics = 9;
    }
}
//! 初始化相机模型和图像大小
Camera::Parameters::Parameters(ModelType modelType,
                               const std::string& cameraName,
                               int w, int h)
 : m_modelType(modelType)
 , m_cameraName(cameraName)
 , m_imageWidth(w)
 , m_imageHeight(h)
{
    switch (modelType)
    {
    case KANNALA_BRANDT:
        m_nIntrinsics = 8;
        break;
    case PINHOLE:
        m_nIntrinsics = 8;
        break;
    case SCARAMUZZA:
        m_nIntrinsics = SCARAMUZZA_CAMERA_NUM_PARAMS;
        break;
    case MEI:
    default:
        m_nIntrinsics = 9;
    }
}

Camera::ModelType&
Camera::Parameters::modelType(void)
{
    return m_modelType;
}

std::string&
Camera::Parameters::cameraName(void)
{
    return m_cameraName;
}

int&
Camera::Parameters::imageWidth(void)
{
    return m_imageWidth;
}

int&
Camera::Parameters::imageHeight(void)
{
    return m_imageHeight;
}

Camera::ModelType
Camera::Parameters::modelType(void) const
{
    return m_modelType;
}

const std::string&
Camera::Parameters::cameraName(void) const
{
    return m_cameraName;
}

int
Camera::Parameters::imageWidth(void) const
{
    return m_imageWidth;
}

int
Camera::Parameters::imageHeight(void) const
{
    return m_imageHeight;
}

int
Camera::Parameters::nIntrinsics(void) const
{
    return m_nIntrinsics;
}

cv::Mat&
Camera::mask(void)
{
    return m_mask;
}

const cv::Mat&
Camera::mask(void) const
{
    return m_mask;
}
/**
 * [Camera::estimateExtrinsics 相机外参估计]
 * @param objectPoints [空间3D点]
 * @param imagePoints  [图像像素点]
 * @param rvec         [相机外参旋转]
 * @param tvec         [相机外参平移]
 */
void
Camera::estimateExtrinsics(const std::vector<cv::Point3f>& objectPoints,
                           const std::vector<cv::Point2f>& imagePoints,
                           cv::Mat& rvec, cv::Mat& tvec) const
{
    std::vector<cv::Point2f> Ms(imagePoints.size());
    for (size_t i = 0; i < Ms.size(); ++i)
    {
      //! 获取相机在归一化平面的坐标
        Eigen::Vector3d P;
        liftProjective(Eigen::Vector2d(imagePoints.at(i).x, imagePoints.at(i).y), P);

        P /= P(2);

        Ms.at(i).x = P(0);
        Ms.at(i).y = P(1);
    }

    // assume unit focal length, zero principal point, and zero distortion
     //！假设内参矩阵是单位阵，求取相机的外参 
    cv::solvePnP(objectPoints, Ms, cv::Mat::eye(3, 3, CV_64F), cv::noArray(), rvec, tvec);
}
/**
 * [Camera::reprojectionDist 求取相机坐标系下两个3D点的重投影距离]
 * @param  P1 [3D点]
 * @param  P2 [3D点]
 * @return    [重投影距离]
 */
double
Camera::reprojectionDist(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2) const
{
    Eigen::Vector2d p1, p2;

    spaceToPlane(P1, p1);
    spaceToPlane(P2, p2);

    return (p1 - p2).norm();
}
/**
 * [Camera::reprojectionError 计算重投影误差,主要是用在相机校准当中]
 * @param  objectPoints   [空间3D点]
 * @param  imagePoints    [图像像素点]
 * @param  rvecs          [相机位置]
 * @param  tvecs          [相机姿态]
 * @param  _perViewErrors [单张图像的误差均值]
 * @return                [得到的重投影误差]
 */
double
Camera::reprojectionError(const std::vector< std::vector<cv::Point3f> >& objectPoints,
                          const std::vector< std::vector<cv::Point2f> >& imagePoints,
                          const std::vector<cv::Mat>& rvecs,
                          const std::vector<cv::Mat>& tvecs,
                          cv::OutputArray _perViewErrors) const
{
    int imageCount = objectPoints.size();
    size_t pointsSoFar = 0;
    double totalErr = 0.0;

     //! 判断该输出的矩阵是否需要计算
    bool computePerViewErrors = _perViewErrors.needed();
    cv::Mat perViewErrors;
    if (computePerViewErrors)
    {
        _perViewErrors.create(imageCount, 1, CV_64F);
        perViewErrors = _perViewErrors.getMat();
    }

    for (int i = 0; i < imageCount; ++i)
    {
        size_t pointCount = imagePoints.at(i).size();

        pointsSoFar += pointCount;
         //！计算由相机位姿和空间3D点投影后的像素点
        std::vector<cv::Point2f> estImagePoints;
        projectPoints(objectPoints.at(i), rvecs.at(i), tvecs.at(i),
                      estImagePoints);
        //！计算误差
        double err = 0.0;
        for (size_t j = 0; j < imagePoints.at(i).size(); ++j)
        {
            err += cv::norm(imagePoints.at(i).at(j) - estImagePoints.at(j));
        }
         //! 计算单张图像的误差均值
        if (computePerViewErrors)
        {
            perViewErrors.at<double>(i) = err / pointCount;
        }

        totalErr += err;
    }

    return totalErr / pointsSoFar;
}
/**
 * [Camera::reprojectionError 世界单个3D点的重投影误差计算]
 * @param  P          [空间3D点]
 * @param  camera_q   [相机姿态]
 * @param  camera_t   [相机位置]
 * @param  observed_p [图像像素点]
 * @return            [重投影误差]
 */
double
Camera::reprojectionError(const Eigen::Vector3d& P,
                          const Eigen::Quaterniond& camera_q,
                          const Eigen::Vector3d& camera_t,
                          const Eigen::Vector2d& observed_p) const
{
    Eigen::Vector3d P_cam = camera_q.toRotationMatrix() * P + camera_t;

    Eigen::Vector2d p;
    spaceToPlane(P_cam, p);

    return (p - observed_p).norm();
}
/**
 * [Camera::projectPoints 将世界坐标系的3D点投影到图像平面上]
 * @param objectPoints [3D坐标点]
 * @param rvec         [相机姿态]
 * @param tvec         [相机位置]
 * @param imagePoints  [投影后的像素点]
 */
void
Camera::projectPoints(const std::vector<cv::Point3f>& objectPoints,
                      const cv::Mat& rvec,
                      const cv::Mat& tvec,
                      std::vector<cv::Point2f>& imagePoints) const
{
    // project 3D object points to the image plane
    imagePoints.reserve(objectPoints.size());

    //double
    //！ 将旋转向量转为旋转矩阵
    cv::Mat R0;
    cv::Rodrigues(rvec, R0);

    Eigen::MatrixXd R(3,3);
    R << R0.at<double>(0,0), R0.at<double>(0,1), R0.at<double>(0,2),
         R0.at<double>(1,0), R0.at<double>(1,1), R0.at<double>(1,2),
         R0.at<double>(2,0), R0.at<double>(2,1), R0.at<double>(2,2);

    Eigen::Vector3d t;
    t << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);

    for (size_t i = 0; i < objectPoints.size(); ++i)
    {
        const cv::Point3f& objectPoint = objectPoints.at(i);

        // Rotate and translate
        //! 转到相机坐标系
        Eigen::Vector3d P;
        P << objectPoint.x, objectPoint.y, objectPoint.z;

        P = R * P + t;
        //! 根据内参投影到图像平面
        Eigen::Vector2d p;
        spaceToPlane(P, p);

        imagePoints.push_back(cv::Point2f(p(0), p(1)));
    }
}

}
