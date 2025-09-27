#ifndef CSatellitePatch_H
#define CSatellitePatch_H

#include <iostream>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>

#include "misc/FH_commen_definition.h"
#include "misc/CoordTransformation.h"
#include "message/CommonDefinitionX.hh"

class Satellite_Patch
{
public:
    Satellite_Patch(std::string satellite_img_path);
    double2D blhToPixelCoord(double2D blh_pos);
    double2D pixelCoordToBlh(double2D pixel_pos);
    void Run(double2D blh_pos, Eigen::Matrix3d R_w_l);
    void Run2(double2D blh_pos, Eigen::Matrix3d R_w_l);
    void Run3(double2D blh_pos, Eigen::Matrix3d R_w_l, cv::Mat &satellite_mask_patch);

    double2D top_left_blh, low_right_blh;
    double map_info_[6];
    cv::Mat satellite_img, satellite_mask_road, satellite_mask_road_cp;
    int ID;

    // for save
    std::string path_common = "/home/guanglei/work/code_v2/Xue/data/meta_2/Hexi_2/";
    std::string str_patch_output_name = path_common + "Satellite_Patch";
    std::string str_mask_output_name = path_common + "Satellite_Mask_Road";
};

#endif // SATELLITE_PATCH_H
