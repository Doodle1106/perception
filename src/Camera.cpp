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

    fsSettings["Rbc"] >> mRbc;
    fsSettings["tbc"] >> mtbc;

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

    mSimulationMode = fsSettings["RUN_SIM"];

    if (mK1.empty() || mK2.empty() || mD1.empty() || mD2.empty() ||
        mR12.empty() || mt12.empty() || mRbc.empty() || mtbc.empty())
    {
        LOG(ERROR) << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return;
    }

    if (!this->ImageAlignInit())
    {
        LOG(ERROR) <<"Camera init failed!"<< endl;
        return;
    }

    mKx = mK1.at<double>(0, 0);
    mKy = mK1.at<double>(1, 1);
    mKxInv = 1.0 / mKx;
    mKyInv = 1.0 / mKy;
    mCx = mK1.at<double>(0, 2);
    mCy = mK1.at<double>(1, 2);
    mBF = mBaseline * mKx;

    mTbc = toTransformationMat(mRbc, mtbc);
    cv::cv2eigen(mTbc, mTbc_eigen);

    mTsim_eigen <<  0.0,  0.0, 1.0, 0.0,
                   -1.0,  0.0, 0.0, 0.0,
                    0.0, -1.0, 0.0, 0.0,
                    0.0,  0.0, 0.0, 0.0;

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
    cout<<"mRbc: "<<endl<<mRbc<<endl;
    cout<<"mtbc: "<<endl<<mtbc<<endl;
    cout<<"mTbc: "<<endl<<mTbc<<endl;
    cout<<"mTbc_eigen: "<<endl<<mTbc_eigen<<endl;
    cout<<"mTsim_eigen: "<<mTsim_eigen<<endl;
    cout<<"mSimulationMode?: "<<mSimulationMode<<endl;
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

void Camera::AddImage(cv::Mat& ImgL, cv::Mat& ImgR, cv::Mat& Twb) {
    if (ImgL.empty() || ImgR.empty())
        LOG(ERROR) << "Added images are empty, neglect!" << endl;

    if (ImgL.channels() == 3)
    {
        cv::cvtColor(ImgL, ImgL, cv::COLOR_RGB2GRAY);
        cv::cvtColor(ImgR, ImgR, cv::COLOR_RGB2GRAY);
    }

    mCurrentDisparity = this->ComputeDisparity(ImgL, ImgR, false);

    this->Disparity2Pointcloud(mCurrentDisparity, Twb);

}

void Camera::AddDisparity(cv::Mat& disparity, cv::Mat& Twb)
{
    LOG(INFO) <<"AddDisparity, camera_id: "<<mCameraID<< endl;

    if (disparity.empty())
        LOG(ERROR) <<"Added disparity empty, neglect!"<< endl;

    this->Disparity2Pointcloud(disparity, Twb);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Camera::AddDepth(cv::Mat& depth, cv::Mat& Twb)
{
    LOG(INFO) <<"AddDisparity, camera_id: "<<mCameraID<< endl;
    cout<<"AddDepth, camera_id: "<<mCameraID<<endl;

    if (depth.empty())
        LOG(ERROR) <<"Added depth empty, neglect!"<< endl;

    return this->Depth2Pointcloud(depth, Twb);
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

pcl::PointCloud<pcl::PointXYZ>::Ptr Camera::Disparity2Pointcloud(cv::Mat &disparity, cv::Mat Twb)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr camera_points(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::PointXYZ> pts;

    for(int i=0; i<disparity.cols; i++)
    {
        for(int j=0; j<disparity.rows; j++)
        {
            float disp = disparity.at<float>(i, j);

            if (isnan(disp) || disp <= 0)
                continue;

            float z = mBF / disp;
            float x = (i - mCx) * z * mKxInv;
            float y = (j - mCy) * z * mKyInv;

            pcl::PointXYZ point(x, y, z);
            camera_points->points.push_back(point);
        }
    }

    cout<<"camera_points: "<<camera_points<<endl;

    //cout<<"total points converted: " <<mCameraPoints.size()<<endl;
    return camera_points;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Camera::Depth2Pointcloud(cv::Mat& depth, cv::Mat Twb)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr camera_points(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::PointXYZ> pts;

    for(int i=0; i<depth.cols; i++)
    {
        for(int j=0; j<depth.rows; j++)
        {
            float dep = depth.at<float>(j, i);

            if (isnan(dep) || dep <= 0)
                continue;

            float x = (i - mCx) * dep * mKxInv;
            float y = (j - mCy) * dep * mKyInv;
            float z = dep;

            pcl::PointXYZ point(x, y, z);
            camera_points->points.push_back(point);
        }
    }

    cout<<"camera_points: "<<camera_points<<endl;


    if(mSimulationMode)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr ros_pts(new pcl::PointCloud <pcl::PointXYZ>);
        pcl::transformPointCloud(*camera_points, *camera_points, mTsim_eigen);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pts(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*camera_points, *transformed_pts, mTbc_eigen);

    Eigen::Matrix<float, 4, 4, Eigen::DontAlign> Twb_eigen;
    cv::cv2eigen(Twb, Twb_eigen);

    pcl::PointCloud<pcl::PointXYZ>::Ptr local_frame_pts(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*transformed_pts, *local_frame_pts, Twb_eigen);

    cout<<"Twb_eigen: "<<Twb_eigen<<endl;

    Eigen::Matrix<float, 4, 4, Eigen::DontAlign> Debug;
    Debug << -1.0,  0.0, 0.0, 0.0,
              0.0, -1.0, 0.0, 0.0,
              0.0,  0.0, 1.0, 0.0,
              0.0,  0.0, 0.0, 0.0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr debug_local_frame_pts(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*local_frame_pts, *debug_local_frame_pts, Debug);

    //return camera_points;
    return debug_local_frame_pts;
}

cv::Mat Camera::ComputeDisparity(cv::Mat& ImgL, cv::Mat& ImgR, bool useCuda)
{
    this->Preprocess(ImgL, ImgR);

    cv::Mat disparity;

    return disparity;
}





