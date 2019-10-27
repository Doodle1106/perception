#include <iostream>

#include <glog/logging.h>
#include "VisionFusionNode.cpp"

int main(int argc, char** argv){

    //google::SetLogDestination(google::GLOG_INFO, "./log_perception_" );
    FLAGS_alsologtostderr = 1;
    FLAGS_minloglevel = 0;
    google::InitGoogleLogging(argv[0]);

    ros::init(argc, argv, "gaas_perception_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    VisionFusionNode* pFusion = new VisionFusionNode(nh, nh_private);

    ros::spin();
    return 0;
}