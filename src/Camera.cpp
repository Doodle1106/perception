//
// Created by gishr on 2019/9/10.
//

#include "../include/Camera.h"

using namespace perception;

Camera::Camera(string& config)
{
    cv::FileStorage fsSettings(config, cv::FileStorage::READ);

    fsSettings["LEFT.K"] >> mK1;
    fsSettings["RIGHT.K"] >> mK2;

    fsSettings["LEFT.D"] >> mD1;
    fsSettings["RIGHT.D"] >> mD2;

    fsSettings["R12"] >> mR12;
    fsSettings["t12"] >> mt12;

    mImageLeftSize.width = fsSettings["LEFT.width"];
    mImageLeftSize.height = fsSettings["LEFT.height"];

    mImageRightSize.width = fsSettings["RIGHT.width"];
    mImageRightSize.height = fsSettings["RIGHT.height"];

    mFinalSize.width = fsSettings["OUTPUT.width"];
    mFinalSize.height = fsSettings["OUTPUT.height"];

    mLeftTopic = string(fsSettings["LEFT_TOPIC"]);
    mRightTopic = string(fsSettings["RIGHT_TOPIC"]);
    mCameraID = fsSettings["CAMERA_ID"];
    mImageType = fsSettings["IMAGE_TYPE"];

    if (mK1.empty() || mK2.empty() || mD1.empty() || mD2.empty() || mR12.empty() || mt12.empty())
    {
        LOG(ERROR) << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return;
    }

    if (!this->ImageAlignInit())
    {
        LOG(ERROR) <<"Camera init failed!"<< endl;
        return;
    }

    LOG(INFO)<<"Camera init succeed!: "<<config<<endl;
};

bool Camera::ImageAlignInit()
{
    if(mR12.empty() || mt12.empty() ||  mK1.empty() || mK2.empty() || mD1.empty() || mD2.empty())
    {
        LOG(ERROR)<<"Loading parameter failed!"<<endl;
        return false;
    }

    cv::stereoRectify(mK1, mD1, mK2, mD2, mImageRightSize, mR12, mt12,
                      mR1, mR2, mP1, mP2, mQ,
                      CV_CALIB_ZERO_DISPARITY, 0, mFinalSize);


    cv::initUndistortRectifyMap(mK1, mD1, mR1, mP1, mFinalSize, CV_32F, lmap1x, lmap1y);
    cv::initUndistortRectifyMap(mK2, mD2, mR2, mP2, mFinalSize, CV_32F, lmap2x, lmap2y);

    return true;
}

void Camera::AddImage(cv::Mat& ImgL, cv::Mat& ImgR) {
    if (ImgL.empty() || ImgR.empty())
        LOG(ERROR) << "Added images are empty, neglect!" << endl;

    if (ImgL.channels() == 3)
    {
        cv::cvtColor(ImgL, ImgL, cv::COLOR_RGB2GRAY);
        cv::cvtColor(ImgR, ImgR, cv::COLOR_RGB2GRAY);
    }

    mCurrentDisparity = this->ComputeDisparity(ImgL, ImgR, false);

    this->Disparity2Pointcloud(mCurrentDisparity, mTic);

}

void Camera::AddDisparity(cv::Mat& disparity)
{
    if (disparity.empty())
        LOG(ERROR) <<"Added disparity empty, neglect!"<< endl;

    this->Disparity2Pointcloud(disparity, mTic);
}

void Camera::ImageAlign(cv::Mat& ImgL, cv::Mat& ImgR)
{
    cv::remap(ImgL, mImgR, lmap1x, lmap1y, cv::INTER_LINEAR);
    cv::remap(ImgR, mImgR, lmap2x, lmap2y, cv::INTER_LINEAR);
}

void Camera::Preprocess(cv::Mat &ImgL, cv::Mat &ImgR)
{
    this->ImageAlign(ImgL, ImgR);
}

void Camera::Disparity2Pointcloud(cv::Mat &disparity, cv::Mat Tic)
{

}

cv::Mat Camera::ComputeDisparity(cv::Mat& ImgL, cv::Mat& ImgR, bool useCuda)
{
    this->Preprocess(ImgL, ImgR);

    cv::Mat disparity;

    return disparity;
}





