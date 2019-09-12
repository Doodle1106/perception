//
// Created by gishr on 2019/9/10.
//

#ifndef PERCEPTION_CAMERA_H
#define PERCEPTION_CAMERA_H

#include "memory"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <iostream>
#include <Eigen/Core>
#include <math.h>
#include <glog/logging.h>

using namespace std;

namespace perception {

    using namespace std;

    class Camera {

    public:

        Camera(string& config);

        void Preprocess(cv::Mat& ImgL, cv::Mat& ImgR);

        bool ImageAlignInit();

        void ImageAlign(cv::Mat& ImgL, cv::Mat& ImgR);

        void Disparity2Pointcloud(cv::Mat& disparity, cv::Mat Tic);

        void Depth2Pointcloud(cv::Mat& depth, cv::Mat Tic);

        cv::Mat ComputeDisparity(cv::Mat& ImgL, cv::Mat& ImgR, bool useCuda);

        void AddImage(cv::Mat& ImgL, cv::Mat& ImgR, cv::Mat& Tic);

        void AddDisparity(cv::Mat& disparity, cv::Mat& Tic);

        void AddDepth(cv::Mat& depth, cv::Mat& Tic);

    private:

        cv::Mat mImgL, mImgR;

        cv::Mat mCurrentDisparity;

        // Rotation matrixs, translation matrixs, camera matrixs, and distortion matrixs
        cv::Mat mK1, mK2, mD1, mD2, mR12, mt12;

        cv::Size mImageLeftSize, mImageRightSize, mFinalSize;

        cv::Mat mR1, mP1, mR2, mP2;

        cv::Mat mQ;

        // Rotation and translation from current camera system to inertial frame
        cv::Mat mRic, mtic, mTic;

        cv::Mat lmap1x, lmap2x, lmap1y, lmap2y;

        std::vector<Eigen::Vector3f> mCameraPoints;
        std::vector<Eigen::Vector3f> mWorldPoints;

        float mKxInv = -1;
        float mKyInv = -1;
        float mCx = -1;
        float mCy = -1;

    public:

        string mLeftTopic, mRightTopic;
        string mDepthTopic;

        int mCameraID = 0;

        // 0 -> stereo gray image, 1 -> disparity
        int mImageType = 0;
    };


}


#endif //PERCEPTION_CAMERA_H
