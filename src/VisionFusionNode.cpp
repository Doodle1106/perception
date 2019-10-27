//
// Created by gishr on 2019/9/11.
//

#include "../include/VisionFusionNode.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>

using namespace perception;
using namespace std;

VisionFusionNode::VisionFusionNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
mNH(nh), mPrivateNH(nh_private)
{

    mPrivateNH.param<string>("mav_name", mMavName, "None");
    mPrivateNH.param<int>("camera_number", mCameraNum, 4);

    mCameraConfigs.resize(mCameraNum);
    mpCameras.resize(mCameraNum);

    LOG(INFO)<<"Number of Cameras Set: "<<mCameraNum<<endl;

    for(int i=0; i<mCameraNum; i++)
    {

        mPrivateNH.param<string>("camera_config_" + to_string(i), mCameraConfigs[i],
                                 "~/software/perception_ws/src/perception/camera_calib/camera_"+ to_string(i) + ".yaml");

        LOG(INFO) <<"Camera id: "<<i<<", config path: "<<mCameraConfigs[i]<< endl;
        mpCameras[i] = make_shared<Camera>(mCameraConfigs[i]);
    }

    LOG(INFO)<<"Total "<<mCameraConfigs.size() <<" cameras found!"<<endl;

    mPointCloudFilter = make_shared<PointCloudFilter>();

    mPointCloudPub = mNH.advertise<sensor_msgs::PointCloud2>("/gi/perception/cloud_in", 1);
    mLocalPoseSub = mNH.subscribe("/mavros/local_position/pose", 1, &VisionFusionNode::PoseCallback, this);

    //camera 0, forward facing, stereo gray image
//    Cam_0_left(mNH, mpCameras[0]->mLeftTopic, 10);
//    Cam_0_right(mNH, mpCameras[0]->mRightTopic, 10);
//    message_filters::Synchronizer<sync_pol> sync(mSyncPol(10), Cam_0_left, Cam_0_right);
//    sync.setMaxIntervalDuration(ros::Duration(0.01));
//    sync.registerCallback(boost::bind(StereoImageCallback, 0, _1, _2), this);

    //camera 0, forward facing, depth image
    LOG(INFO)<<mpCameras[0]->mDepthTopic<<std::endl;
    mSubCamera_0 = mNH.subscribe<sensor_msgs::Image>(mpCameras[0]->mDepthTopic, 5,
                                 boost::bind(&VisionFusionNode::DepthCallback, this, _1, 0));

    //camera 1,  facing towards right, stereo gray image
//    Cam_1_left(mNH, mpCameras[1]->mLeftTopic, 10);
//    Cam_1_right(mNH, mpCameras[1]->mRightTopic, 10);
//    message_filters::Synchronizer<sync_pol> sync(mSyncPol(10), Cam_1_left, Cam_1_right);
//    sync.setMaxIntervalDuration(ros::Duration(0.01));
//    sync.registerCallback(boost::bind(StereoImageCallback, 1, _1, _2), this);

    //camera 1, facing towards right, disparity image
//    std::cout<<mpCameras[1]->mDepthTopic<<std::endl;
//    mSubCamera_1 = mNH.subscribe<sensor_msgs::Image>(mpCameras[1]->mDepthTopic, 5,
//                                 boost::bind(&VisionFusionNode::DepthCallback, this, _1, 1));

//    //camera 2, backward facing, stereo gray image
//    Cam_2_left(mNH, mpCameras[2]->mLeftTopic, 10);
//    Cam_2_right(mNH, mpCameras[2]->mRightTopic, 10);
//    message_filters::Synchronizer<sync_pol> sync(mSyncPol(10), Cam_2_left, Cam_2_right);
//    sync.setMaxIntervalDuration(ros::Duration(0.01));
//    sync.registerCallback(boost::bind(StereoImageCallback, 2, _1, _2), this);
//
//    //camera 2, backward facing, disparity image
//    std::cout<<mpCameras[2]->mDepthTopic<<std::endl;
//    mSubCamera_2 = mNH.subscribe<sensor_msgs::Image>(mpCameras[2]->mDepthTopic, 5,
//                                                     boost::bind(&VisionFusionNode::DepthCallback, this, _1, 2));

//    //camera 3, facing towards left, stereo gray image
//    Cam_3_left(mNH, mpCameras[i]->mLeftTopic, 10);
//    Cam_3_right(mNH, mpCameras[i]->mRightTopic, 10);
//    message_filters::Synchronizer<sync_pol> sync(mSyncPol(10), Cam_3_left, Cam_3_right);
//    sync.setMaxIntervalDuration(ros::Duration(0.01));
//    sync.registerCallback(boost::bind(StereoImageCallback, 3, _1, _2), this);
//
//    //camera 3, facing towards left, disparity image
//    std::cout<<mpCameras[3]->mDepthTopic<<std::endl;
//    mSubCamera_3 = mNH.subscribe<sensor_msgs::Image>(mpCameras[3]->mDepthTopic, 5,
//                                                     boost::bind(&VisionFusionNode::DepthCallback, this, _1, 3));


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
    mpCameras[camera_id]->AddImage(imLeftRect, imRightRect, mTwb);
}

void VisionFusionNode::DisparityCallback(const sensor_msgs::ImageConstPtr& msgLeft, int camera_id)
{
    LOG(INFO) <<"DisparityCallback, camera_id: "<<camera_id<< endl;
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
    mpCameras[camera_id]->AddDisparity(imLeftRect, mTwb);
}

void VisionFusionNode::DepthCallback(const sensor_msgs::ImageConstPtr& msgLeft, int camera_id)
{
    LOG(INFO) <<"DepthCallback, camera_id: "<<camera_id<< endl;

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

    cv::Mat imLeftRect = cv_ptrLeft->image;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pPC;
    pPC = mpCameras[camera_id]->AddDepth(imLeftRect, mTwb);
    LOG(INFO)<<"pPC size: "<<pPC->points.size()<<endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
    cloud_source = pPC;
    cloud_source->width = 1;
    cloud_source->height = pPC->points.size();
    pcl::io::savePCDFileASCII ("/home/shr/software/perception_ws/src/perception/test_pcd.pcd", *cloud_source);

    sensor_msgs::PointCloud2 rosPC;
    mPointCloudFilter->PointCloudXYZtoROSPointCloud2(*pPC, rosPC);
    LOG(INFO)<<"rosPC size: "<<rosPC.data.size()<<endl;

    rosPC.header.frame_id = "map";
    mPointCloudPub.publish(rosPC);
}

void VisionFusionNode::PoseCallback(const geometry_msgs::PoseStamped& pose)
{
    mLocalPose = pose;

    Eigen::Quaterniond q(pose.pose.orientation.x, pose.pose.orientation.y,
                         pose.pose.orientation.z, pose.pose.orientation.w);
    Eigen::Vector3d translation(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

    cout<<"q and t: "<<", "<<translation<<endl;

    Sophus::SE3d se3d(q, translation);

    auto r = se3d.rotationMatrix();
    auto t = se3d.translation();

    cout<<t(0, 0)<<t(1, 0)<<t(2, 0)<<t(0, 0)<<endl;

    cv::Mat_<double> T(4,4);
    T << r(0, 0), r(0, 1), r(0, 2), t(0, 0),
         r(1, 0), r(1, 1), r(1, 2), t(1, 0),
         r(2, 0), r(2, 1), r(2, 2), t(2, 0),
         0.0,     0.0,     0.0,     1.0;

    cv::eigen2cv(r, mRwb);
    cv::eigen2cv(t, mtwb);

    mTwb = T;
}
