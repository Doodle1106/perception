//
// Created by gishr on 2019/9/11.
//

#ifndef PERCEPTION_VISIONFUSION_H
#define PERCEPTION_VISIONFUSION_H

#include "Camera.h"

#include <vector>
#include <memory>
#include <glog/logging.h>

namespace perception {


    class VisionFusion {

    public:

        VisionFusion(vector<shared_ptr<Camera> > camera_ptrs);

        void AddImage(int idx, cv::Mat& imgL, cv::Mat& imgR);

        void AddDisparity(int idx, cv::Mat& disparity);

    private:

        int mCameraNum;

        vector<shared_ptr<Camera> > mpCameras;


    };

}

#endif //PERCEPTION_VISIONFUSION_H
