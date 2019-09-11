//
// Created by gishr on 2019/9/11.
//

#include "../include/VisionFusionNode.h"

using namespace perception;

#include <iostream>
using namespace std;

VisionFusionNode::VisionFusionNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
mNH(nh), mPrivateNH(nh_private)
{

    mPrivateNH.param<string>("mav_name", mMavName, "None");
    mPrivateNH.param<int>("camera_number", mCameraNum, 2);

    mCameraConfigs.resize(mCameraNum);
    mpCameras.resize(mCameraNum);

    LOG(INFO)<<"mCameraNum: "<<mCameraNum<<endl;
    std::cout<<"mCameraNum: "<<mCameraNum<<std::endl;

    for(int i=0; i<mCameraNum; i++)
    {
        LOG(INFO)<<"/home/gishr/software/perception/src/perception/camera_calib/camera_"+ to_string(i) + ".yaml"<<endl;
        std::cout<<"/home/gishr/software/perception/src/perception/camera_calib/camera_"+ to_string(i) + ".yaml"<<std::endl;

        mPrivateNH.param<string>("camera_config_" + to_string(i), mCameraConfigs[i],
                                 "/home/gishr/software/perception/src/perception/camera_calib/camera_"+ to_string(i) + ".yaml");

        LOG(INFO) <<"Camera id: "<<i<<", config path: "<<mCameraConfigs[i]<< endl;
        mpCameras[i] = make_shared<Camera>(mCameraConfigs[i]);
    }

    LOG(INFO)<<"Total "<<mCameraConfigs.size() <<" cameras found!"<<endl;
    std::cout<<"Total "<<mCameraConfigs.size() <<" cameras found!"<<std::endl;

    //camera 0, forward facing, stereo gray image
//    Cam_0_left(mNH, mpCameras[0]->mLeftTopic, 10);
//    Cam_0_right(mNH, mpCameras[0]->mRightTopic, 10);
//    message_filters::Synchronizer<sync_pol> sync(mSyncPol(10), Cam_0_left, Cam_0_right);
//    sync.setMaxIntervalDuration(ros::Duration(0.01));
//    sync.registerCallback(boost::bind(StereoImageCallback, 0, _1, _2), this);

    //camera 0, forward facing, disparity image
    std::cout<<mpCameras[0]->mLeftTopic<<std::endl;

    mSubCamera_0 = mNH.subscribe<sensor_msgs::Image>(mpCameras[0]->mLeftTopic, 5,
                                 boost::bind(&VisionFusionNode::DisparityCallback, this, _1, 0));

    //camera 1,  facing towards right, stereo gray image
//    Cam_1_left(mNH, mpCameras[1]->mLeftTopic, 10);
//    Cam_1_right(mNH, mpCameras[1]->mRightTopic, 10);
//    message_filters::Synchronizer<sync_pol> sync(mSyncPol(10), Cam_1_left, Cam_1_right);
//    sync.setMaxIntervalDuration(ros::Duration(0.01));
//    sync.registerCallback(boost::bind(StereoImageCallback, 1, _1, _2), this);

    //camera 1, facing towards right, disparity image
    std::cout<<mpCameras[1]->mLeftTopic<<std::endl;
    mSubCamera_1 = mNH.subscribe<sensor_msgs::Image>(mpCameras[1]->mLeftTopic, 5,
                                 boost::bind(&VisionFusionNode::DisparityCallback, this, _1, 1));

//    //camera 2, backward facing, stereo gray image
//    Cam_2_left(mNH, mpCameras[2]->mLeftTopic, 10);
//    Cam_2_right(mNH, mpCameras[2]->mRightTopic, 10);
//    message_filters::Synchronizer<sync_pol> sync(mSyncPol(10), Cam_2_left, Cam_2_right);
//    sync.setMaxIntervalDuration(ros::Duration(0.01));
//    sync.registerCallback(boost::bind(StereoImageCallback, 2, _1, _2), this);
//
//    //camera 2, backward facing, disparity image
//    mSubCamera_2 = nh.subscribe<sensor_msgs::Image> (mpCameras[2]->mLeftTopic, 5,
//                                                     boost::bind(DisparityCallback, 2, _1),
//                                                     this,ros::TransportHints().tcpNoDelay());
//
//    //camera 3, facing towards left, stereo gray image
//    Cam_3_left(mNH, mpCameras[i]->mLeftTopic, 10);
//    Cam_3_right(mNH, mpCameras[i]->mRightTopic, 10);
//    message_filters::Synchronizer<sync_pol> sync(mSyncPol(10), Cam_3_left, Cam_3_right);
//    sync.setMaxIntervalDuration(ros::Duration(0.01));
//    sync.registerCallback(boost::bind(StereoImageCallback, 3, _1, _2), this);
//
//    //camera 3, facing towards left, disparity image
//    mSubCamera_3 = nh.subscribe<sensor_msgs::Image> (mpCameras[3]->mLeftTopic, 5,
//                                                     boost::bind(DisparityCallback, 3, _1),
//                                                     this,ros::TransportHints().tcpNoDelay());


}


void VisionFusionNode::StereoImageCallback(const int camera_id,
                                           const sensor_msgs::ImageConstPtr& msgLeft,
                                           const sensor_msgs::ImageConstPtr& msgRight)
{
    if(camera_id < 0 || camera_id > mCameraNum)
    {
        LOG(ERROR)<<"Camera Idx wrong! Neglect!"<<endl;
        return;
    }

    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat imLeftRect, imRightRect;
    imLeftRect = cv_ptrLeft->image;
    imRightRect = cv_ptrRight->image;
    mpCameras[camera_id]->AddImage(imLeftRect, imRightRect);
}

void VisionFusionNode::DisparityCallback(const sensor_msgs::ImageConstPtr& msgLeft, int camera_id)
{
    std::cout<<"DisparityCallback, camera_id: "<<camera_id<<std::endl;

    if(camera_id < 0 || camera_id > mCameraNum)
    {
        LOG(ERROR) << "Camera Idx wrong! Neglect!" << endl;
        return;
    }

    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat imLeftRect;
    imLeftRect = cv_ptrLeft->image;
    mpCameras[camera_id]->AddDisparity(imLeftRect);
}

void VisionFusionNode::PoseCallback(const geometry_msgs::PoseStamped& pose)
{
    mLocalPose = pose;
}
