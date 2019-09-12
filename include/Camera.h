//
// Created by gishr on 2019/9/10.
//

#ifndef PERCEPTION_CAMERA_H
#define PERCEPTION_CAMERA_H

#include "memory"
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <iostream>
#include <math.h>
#include <glog/logging.h>
#include <PointCloudFilter.h>
#include <pcl/common/transforms.h>

namespace perception {

    using namespace std;

    class Camera {

    public:

//        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Camera(string& config);

        void Preprocess(cv::Mat& ImgL, cv::Mat& ImgR);

        bool ImageAlignInit();

        void ImageAlign(cv::Mat& ImgL, cv::Mat& ImgR);

        pcl::PointCloud<pcl::PointXYZ>::Ptr Disparity2Pointcloud(cv::Mat& disparity, cv::Mat Twb);

        pcl::PointCloud<pcl::PointXYZ>::Ptr Depth2Pointcloud(cv::Mat& depth, cv::Mat Twb);

        cv::Mat ComputeDisparity(cv::Mat& ImgL, cv::Mat& ImgR, bool useCuda);

        void AddImage(cv::Mat& ImgL, cv::Mat& ImgR, cv::Mat& Twb);

        void AddDisparity(cv::Mat& disparity, cv::Mat& Twb);

        pcl::PointCloud<pcl::PointXYZ>::Ptr AddDepth(cv::Mat& depth, cv::Mat& Twb);

        inline cv::Mat toTransformationMat(cv::Mat& r, cv::Mat t)
        {
            cv::Mat_<double> T(4,4);
            T << r.at<double>(0, 0), r.at<double>(0, 1), r.at<double>(0, 2), t.at<double>(0, 0),
                 r.at<double>(1, 0), r.at<double>(1, 1), r.at<double>(1, 2), t.at<double>(0, 1),
                 r.at<double>(2, 0), r.at<double>(2, 1), r.at<double>(2, 2), t.at<double>(0, 2),
                 0.0,     0.0,     0.0,     1.0;

            return T;
        }

        // Rotation and translation from current camera system to inertial frame
        cv::Mat mRbc, mtbc, mTbc;

        // on some platforms it is required to add Eigen::DontAlign
        Eigen::Matrix<float, 4, 4, Eigen::DontAlign> mTbc_eigen;
//        Eigen::Matrix4f mTbc_eigen;
//        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> mTbc_eigen;

    private:

        cv::Mat mImgL, mImgR;

        cv::Mat mCurrentDisparity;

        // Rotation matrixs, translation matrixs, camera matrixs, and distortion matrixs
        cv::Mat mK1, mK2, mD1, mD2, mR12, mt12;

        cv::Size mImageLeftSize, mImageRightSize, mFinalSize;

        cv::Mat mR1, mP1, mR2, mP2;

        cv::Mat mQ;

        cv::Mat lmap1x, lmap2x, lmap1y, lmap2y;

        pcl::PointCloud<pcl::PointXYZ> mCameraPoints;
        pcl::PointCloud<pcl::PointXYZ> mWorldPoints;

        float mBaseline = -1;
        float mKxInv = -1;
        float mKyInv = -1;
        float mCx = -1;
        float mCy = -1;
        float mKx = -1;
        float mKy = -1;
        float mBF = -1;

    public:

        string mLeftTopic, mRightTopic;
        string mDepthTopic;

        int mCameraID = 0;

        // 0 -> stereo gray image, 1 -> disparity
        int mImageType = 0;


    };


}


#endif //PERCEPTION_CAMERA_H
