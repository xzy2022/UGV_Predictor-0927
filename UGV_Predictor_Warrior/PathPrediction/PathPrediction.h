#ifndef PathPrediction_H
#define PathPrediction_H

#include <torch/script.h> // torch::jit::script::Module
#include <torch/torch.h>  // Various tensor utilities
#include <iostream>
#include <memory>
#include "ros/ros.h"
#include "iostream"
#include "world_state/Pose6D.h"
#include "vector"
#include "world_state/TerrainMapComplex.h"
#include "torch/script.h"
#include "torch/csrc/api/include/torch/torch.h"
#include "sys/time.h"
#include "geometry_msgs/Point.h"
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include "CSatellitePatch/CSatellitePatch.h"
#include "misc/FH_commen_definition.h"
#include "misc/Colors.h"

class PathPrediction
{
public:
    PathPrediction(ros::NodeHandle p_node_);
    ~PathPrediction();

    ros::Subscriber ros_sub_terrain;
    ros::Publisher ros_pub_path;
    ros::NodeHandle p_node;
    world_state::TerrainMapComplex *msg;
    sensor_msgs::PointCloud path_predict;
    Satellite_Patch *satellite_patch;

    void Initialize();
    void Run();
    void PathHandler(const world_state::TerrainMapComplex::ConstPtr &terrainmsg);
    torch::Tensor ExtractSubtensor(const boost::array<float, 250000> &array, int width, int height);
    torch::Tensor MatToTensor(const cv::Mat &mat);
    torch::Tensor NormalizeFeatureMap(torch::Tensor input_feature_map);
    void VisMaskRoad(cv::Mat mask_road);
    void VisPredictPath(cv::Mat elevation, bool b_norm, std::string input_win_name, torch::Tensor predict_path);
};

#endif // PathPrediction_H
