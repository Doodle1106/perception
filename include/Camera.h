//
// Created by gishr on 2019/9/10.
//

#ifndef PERCEPTION_CAMERA_H
#define PERCEPTION_CAMERA_H

#include "memory"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <iostream>
#include <glog/logging.h>


namespace perception {

    using namespace std;

    class Camera {

    public:

        Camera(string& config);

        void Preprocess(cv::Mat& ImgL, cv::Mat& ImgR);

        bool ImageAlignInit();

        void ImageAlign(cv::Mat& ImgL, cv::Mat& ImgR);

        void Disparity2Pointcloud(cv::Mat& disparity, cv::Mat Tic);

        cv::Mat ComputeDisparity(cv::Mat& ImgL, cv::Mat& ImgR, bool useCuda);

        void AddImage(cv::Mat& ImgL, cv::Mat& ImgR);

        void AddDisparity(cv::Mat& disparity);

    private:

        cv::Mat mImgL, mImgR;

        cv::Mat mCurrentDisparity;

        // Rotation matrixs, translation matrixs, camera matrixs, and distortion matrixs
        cv::Mat mR12, mt12,  mK1, mK2, mD1, mD2;

        cv::Size mImageLeftSize, mImageRightSize, mFinalSize;

        cv::Mat mR1, mP1, mR2, mP2;

        cv::Mat mQ;

        // Rotation and translation from current camera system to inertial frame
        cv::Mat mRic, mtic, mTic;

        cv::Mat lmap1x, lmap2x, lmap1y, lmap2y;

    public:

        string mLeftTopic, mRightTopic;
        int mCameraID = 0;

        // 0 -> stereo gray image, 1 -> disparity
        int mImageType = 0;
    };


}


#endif //PERCEPTION_CAMERA_H