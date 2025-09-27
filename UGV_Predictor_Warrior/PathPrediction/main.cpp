
#include <ros/ros.h>

#include "PathPrediction.h"

int main(int argc, char *argv[])
{

    std::string node_name = "PathPrediction";
    ros::init(argc, argv, node_name);
    ROS_INFO("NodeName: %s", node_name.c_str());

    ros::NodeHandle p_node;

    PathPrediction *path_prediction = new PathPrediction(p_node);
    path_prediction->Run();

    return 0;
    //    ros::MultiThreadedSpinner spinner(3);
    //    spinner.spin();
    //    ros::spin();
}
