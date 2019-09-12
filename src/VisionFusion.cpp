//
// Created by gishr on 2019/9/11.
//

#include "VisionFusion.h"

#include <utility>


using namespace perception;

VisionFusion::VisionFusion(vector<shared_ptr<Camera> > camera_ptrs)
{
    mpCameras = std::move(camera_ptrs);
}

void VisionFusion::AddImage(int idx, cv::Mat& imgL, cv::Mat& imgR)
{
    if (idx < 0 || idx > mCameraNum)
        LOG(ERROR) <<"Camera idx: "<< idx <<" doesn't exit!"<< endl;

    if (imgL.empty() || imgR.empty())
        LOG(ERROR) <<"Image empty!"<< endl;

    mpCameras[idx]->AddImage(imgL, imgR, mTic);
}

void VisionFusion::AddDisparity(int idx, cv::Mat &disparity)
{
    if (idx < 0 || idx > mCameraNum)
        LOG(ERROR) <<"Camera idx: "<< idx <<" doesn't exit!"<< endl;

    if (disparity.empty())
        LOG(ERROR) <<"Disparity empty!"<< endl;

    mpCameras[idx]->AddDisparity(disparity, mTic);
}

void VisionFusion::AddDepth(int idx, cv::Mat& depth)
{

    if (idx < 0 || idx > mCameraNum)
        LOG(ERROR) <<"Camera idx: "<< idx <<" doesn't exit!"<< endl;

    if (depth.empty())
        LOG(ERROR) <<"Depth empty!"<< endl;

    mpCameras[idx]->AddDepth(depth, mTic);
}


