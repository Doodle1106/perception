//
// Created by gishr on 2019/9/11.
//

#ifndef PERCEPTION_VISIONFUSIONNODE_H
#define PERCEPTION_VISIONFUSIONNODE_H

#include "VisionFusion.h"
#include "include/Camera.h"

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CompanionProcessStatus.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/duration.h>
#include <sensor_msgs/Image.h>
#include <string>

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Transform.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include "se3.hpp"
#include "so3.hpp"

namespace perception {

    class VisionFusionNode{

    public:

        VisionFusionNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

        void StereoImageCallback(const int camera_id, const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

        void DisparityCallback(const sensor_msgs::ImageConstPtr& msgLeft, int camera_id);

        void DepthCallback(const sensor_msgs::ImageConstPtr& msgLeft, int camera_id);

        void PoseCallback(const geometry_msgs::PoseStamped& pose);

    private:

        ros::NodeHandle mNH;
        ros::NodeHandle mPrivateNH;
        ros::Subscriber mLocalPoseSub;

        int mCameraNum;
        string mMavName;
        vector<string> mCameraConfigs;
        vector<shared_ptr<Camera> > mpCameras;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> mSyncPol;
        typedef message_filters::Subscriber<sensor_msgs::Image> mImageFilterSub;

        mImageFilterSub Cam_0_left, Cam_0_right;
        mImageFilterSub Cam_1_left, Cam_1_right;
        mImageFilterSub Cam_2_left, Cam_2_right;
        mImageFilterSub Cam_3_left, Cam_3_right;

        ros::Subscriber mSubCamera_0;
        ros::Subscriber mSubCamera_1;
        ros::Subscriber mSubCamera_2;
        ros::Subscriber mSubCamera_3;

        cv::Mat mTic, mRic, mtic;

    private:

        cv_bridge::CvImageConstPtr cv_ptrLeft;
        cv_bridge::CvImageConstPtr cv_ptrRight;

        geometry_msgs::PoseStamped mLocalPose;

    };

}
#endif //PERCEPTION_VISIONFUSIONNODE_H
