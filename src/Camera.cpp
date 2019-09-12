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
    mDepthTopic = string(fsSettings["DEPTH_TOPIC"]);

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

    mKxInv = 1.0 / mK1.at<double>(0, 0);
    mKyInv = 1.0 / mK1.at<double>(1, 1);
    mCx = mK1.at<double>(0, 2);
    mCy = mK1.at<double>(1, 2);

    LOG(INFO)<<"Camera init succeed!: "<<config<<endl;

    cout<<"mK1: "<<endl<<mK1<<endl;
    cout<<"mK2: "<<endl<<mK2<<endl;
    cout<<"mD1: "<<endl<<mD1<<endl;
    cout<<"mD2: "<<endl<<mD2<<endl;
    cout<<"mR12: "<<endl<<mR12<<endl;
    cout<<"mt12: "<<endl<<mt12<<endl;
    cout<<"mKxInv: "<<endl<<mKxInv<<endl;
    cout<<"mKyInv: "<<endl<<mKyInv<<endl;
    cout<<"mCx: "<<endl<<mCx<<endl;
    cout<<"mCy: "<<endl<<mCy<<endl;

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

void Camera::AddImage(cv::Mat& ImgL, cv::Mat& ImgR, cv::Mat& Tic) {
    if (ImgL.empty() || ImgR.empty())
        LOG(ERROR) << "Added images are empty, neglect!" << endl;

    if (ImgL.channels() == 3)
    {
        cv::cvtColor(ImgL, ImgL, cv::COLOR_RGB2GRAY);
        cv::cvtColor(ImgR, ImgR, cv::COLOR_RGB2GRAY);
    }

    mCurrentDisparity = this->ComputeDisparity(ImgL, ImgR, false);

    this->Disparity2Pointcloud(mCurrentDisparity, Tic);

}

void Camera::AddDisparity(cv::Mat& disparity, cv::Mat& Tic)
{
    LOG(INFO) <<"AddDisparity, camera_id: "<<mCameraID<< endl;

    if (disparity.empty())
        LOG(ERROR) <<"Added disparity empty, neglect!"<< endl;

    this->Disparity2Pointcloud(disparity, Tic);
}

void Camera::AddDepth(cv::Mat& depth, cv::Mat& Tic)
{
    LOG(INFO) <<"AddDisparity, camera_id: "<<mCameraID<< endl;
    cout<<"AddDepth, camera_id: "<<mCameraID<<endl;

    if (depth.empty())
        LOG(ERROR) <<"Added depth empty, neglect!"<< endl;

    this->Depth2Pointcloud(depth, Tic);

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

void Camera::Depth2Pointcloud(cv::Mat& depth, cv::Mat Tic)
{
    for(int i=0; i<mImageLeftSize.width; i++)
    {
        for(int j=0; j<mImageLeftSize.height; j++)
        {
            float dep = depth.at<float>(i, j);

            if (isnan(dep) || dep == 0)
                continue;

            float x = (i - mCx) * dep * mKxInv;
            float y = (j - mCy) * dep * mKyInv;
            float z = dep;

            cout<<"x, y, z: "<<x<<", "<<y<<", "<<z<<endl;
            mCameraPoints.emplace_back(Eigen::Vector3f(x, y ,z));
        }
    }
    cout<<"total points converted: " <<mCameraPoints.size()<<endl;

}

cv::Mat Camera::ComputeDisparity(cv::Mat& ImgL, cv::Mat& ImgR, bool useCuda)
{
    this->Preprocess(ImgL, ImgR);

    cv::Mat disparity;

    return disparity;
}





