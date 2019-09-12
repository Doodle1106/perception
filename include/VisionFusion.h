//
// Created by gishr on 2019/9/11.
//

#ifndef PERCEPTION_VISIONFUSION_H
#define PERCEPTION_VISIONFUSION_H

#include "Camera.h"

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include <glog/logging.h>

namespace perception {


    class VisionFusion {

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        VisionFusion(vector<shared_ptr<Camera> > camera_ptrs);

        void AddImage(int idx, cv::Mat& imgL, cv::Mat& imgR);

        void AddDisparity(int idx, cv::Mat& disparity);

        void AddDepth(int idx, cv::Mat& depth);

    private:

        int mCameraNum;

        vector<shared_ptr<Camera> > mpCameras;

        cv::Mat mTic;

    };

}

#endif //PERCEPTION_VISIONFUSION_H
