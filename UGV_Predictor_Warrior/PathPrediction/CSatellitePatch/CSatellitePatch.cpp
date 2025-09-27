#include <iostream>
#include <sys/stat.h> // 该头文件包含 access 和 mkdir 函数的声明
#include <unistd.h>   // 该头文件包含 F_OK 常量

#include "CSatellitePatch.h"

inline bool directoryExists(const std::string &path)
{
    // 使用 access 函数检查目录是否存在
    return access(path.c_str(), F_OK) == 0;
}

inline void createDirectory(const std::string &path)
{
    // 使用 mkdir 函数创建目录
    int status = mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    if (status == 0)
    {
        std::cout << "Directory created successfully: " << path << std::endl;
    }
    else
    {
        std::cerr << "Failed to create directory: " << path << std::endl;
    }
}

inline void createDirectoriesRecursively(const std::string &path)
{
    size_t pos = 0;
    while ((pos = path.find('/', pos)) != std::string::npos)
    {
        std::string sub_path = path.substr(0, pos);
        if (!sub_path.empty() && !directoryExists(sub_path))
        {
            createDirectory(sub_path);
        }
        ++pos;
    }

    // 处理最后一级目录
    if (!path.empty() && !directoryExists(path))
    {
        createDirectory(path);
    }
}

Satellite_Patch::Satellite_Patch(std::string satellite_img_path)
{

    //     school
    //        std::string img_path = satellite_img_path + "/school.png";
    //        std::string mask_path = satellite_img_path + "/school_mask.png";
    //        top_left_blh.x = 112.98897743225098; top_left_blh.y =  28.231828756298253;
    //        low_right_blh.x = 112.99929857254027; low_right_blh.y = 28.217649000889331;

    ////     school
    //    std::string img_path = satellite_img_path + "/school_whole.png";
    //    std::string mask_path = satellite_img_path + "/school_whole_mask_path.png";
    //    top_left_blh.x = 112.986487; top_left_blh.y =  28.233505;
    //    low_right_blh.x = 112.998883; low_right_blh.y = 28.215530;

    //    // ALS
    //    std::string img_path = satellite_img_path + "/ALS_whole.png";
    //    std::string mask_path = satellite_img_path + "/ALS_whole_mask_path.png";
    //    top_left_blh.x = 105.5489493100000; top_left_blh.y = 39.33553642000000;
    //    low_right_blh.x = 105.6651940900000; low_right_blh.y = 39.245624819999996;

    // Hexi
    //    std::string img_path = satellite_img_path + "/Hexi_new.png";
    //    std::string mask_path = satellite_img_path + "/Hexi_new_A.png";
    //    top_left_blh.x = 112.86140157150000; top_left_blh.y = 28.11529467450002;
    //    low_right_blh.x = 112.8736324445999; low_right_blh.y = 28.105016449500001;

    // Hexi_gl
    std::string img_path = satellite_img_path + "/Hexi_gl/Hexi_gl.png";
    std::string mask_path = satellite_img_path + "/Hexi_gl/Hexi_gl_mask.png";
    top_left_blh.x = 112.86814928054808;
    top_left_blh.y = 28.107692088513648;
    low_right_blh.x = 112.87246763706206;
    low_right_blh.y = 28.10562665626037;

    //    std::string img_path = satellite_img_path + "/KITTI_00.png";
    //    std::string mask_path = satellite_img_path + "/KITTI_08_mask_path_C.png";
    //    // KITTI_00, 07, 08
    //    top_left_blh.x = 8.387331962585; top_left_blh.y = 48.990806344103;
    //    low_right_blh.x = 8.406043052673; low_right_blh.y = 48.977682295393;
    //    // KITTI_02
    ////    top_left_blh.x = 8.465394973755; top_left_blh.y = 49.003529548645;
    ////    low_right_blh.x = 8.495046794415; low_right_blh.y = 48.981344997883;
    //    // KITTI_05
    ////    top_left_blh.x = 8.388833999634; top_left_blh.y = 49.055585861206;
    ////    low_right_blh.x = 8.405739963055; low_right_blh.y = 49.039967358112;
    //    // KITTI_09
    ////    top_left_blh.x = 8.471660614014; top_left_blh.y = 48.975591659546;
    ////    low_right_blh.x = 8.487665355206; low_right_blh.y = 48.960571289062;
    satellite_img = cv::imread(img_path, 1);
    satellite_mask_road = cv::imread(mask_path, 1);
    satellite_mask_road.copyTo(satellite_mask_road_cp);

    map_info_[0] = top_left_blh.x;
    map_info_[1] = (low_right_blh.x - top_left_blh.x) / satellite_img.cols; // 每个pixel的经度差，+
    map_info_[2] = 0;
    map_info_[3] = top_left_blh.y;
    map_info_[4] = 0;
    map_info_[5] = (low_right_blh.y - top_left_blh.y) / satellite_img.rows; // 每个pixel的纬度差，-
    ID = 0;

    // createDirectoriesRecursively(str_mask_output_name);
    // createDirectoriesRecursively(str_patch_output_name);
}

double2D Satellite_Patch::blhToPixelCoord(double2D blh_pos)
{
    double2D pixel_pos;
    double dTemp = map_info_[1] * map_info_[5] - map_info_[2] * map_info_[4];

    pixel_pos.x = (map_info_[5] * (blh_pos.x - map_info_[0]) - map_info_[2] * (blh_pos.y - map_info_[3])) / dTemp;
    pixel_pos.y = (map_info_[1] * (blh_pos.y - map_info_[3]) - map_info_[4] * (blh_pos.x - map_info_[0])) / dTemp;

    return pixel_pos;
}

double2D Satellite_Patch::pixelCoordToBlh(double2D pixel_pos)
{
    double2D blh_pos;
    blh_pos.x = map_info_[0] + pixel_pos.x * map_info_[1] + pixel_pos.y * map_info_[2];
    blh_pos.y = map_info_[3] + pixel_pos.x * map_info_[4] + pixel_pos.y * map_info_[5];
    return blh_pos;
}

void Satellite_Patch::Run(double2D blh_pos, Eigen::Matrix3d R_w_l)
{
    // add error!
    //    double2D current_xy = blh2xy(blh_pos);
    //    double error = 5;
    //    double2D current_xy_with_error;
    //    current_xy_with_error.x = current_xy.x - error;
    //    current_xy_with_error.y = current_xy.y - error;
    //    double2D current_blh_with_error = xy2blh(current_xy_with_error);
    //    double2D pixel_pos_center = blhToPixelCoord(current_blh_with_error);

    // local window pixel
    //    经纬度转像素
    double2D pixel_pos_center = blhToPixelCoord(blh_pos);

    double resolution = 0.297725; // KITTI: 0.297725
    int local_window_rows = ceil(40 / resolution);
    int local_window_cols = ceil(40 / resolution);
    // ALS: (122, 154)
    // KITTI: (135, 135)
    if (local_window_rows != 135 || local_window_cols != 135)
        std::cout << "error size: " << local_window_rows << " ," << local_window_cols << std::endl;

    int whole_rows = 2 * local_window_rows + 1;
    int whole_cols = 2 * local_window_cols + 1;

    cv::Mat satellite_patch = cv::Mat::zeros(whole_rows, whole_cols, CV_8UC3);
    cv::Mat satellite_mask_patch = cv::Mat::zeros(whole_rows, whole_cols, CV_8UC3);
    int min_r = pixel_pos_center.y - local_window_rows;
    int max_r = pixel_pos_center.y + local_window_rows;
    int min_c = pixel_pos_center.x - local_window_cols;
    int max_c = pixel_pos_center.x + local_window_cols;

    int center_idx = whole_rows / 2 + 1;
    for (int r = 0; r < whole_rows; r++)
    {
        for (int c = 0; c < whole_cols; c++)
        {
            double global_x = (c - center_idx) * resolution;
            double global_y = (r - center_idx) * resolution;
            Eigen::Vector3d global_grid, local_grid;
            global_grid << global_x, global_y, 1;
            local_grid = R_w_l.inverse() * global_grid;
            double local_c = local_grid(0) / resolution + center_idx;
            double local_r = local_grid(1) / resolution + center_idx;
            int global_r = round(local_r + min_r);
            int global_c = round(local_c + min_c);
            satellite_patch.at<cv::Vec3b>(r, c) = satellite_img.at<cv::Vec3b>(global_r, global_c);
            satellite_mask_patch.at<cv::Vec3b>(r, c) = satellite_mask_road.at<cv::Vec3b>(global_r, global_c);
        }
    }

    //    for(int r=min_r; r<=min_r+local_window_rows; r++) {
    //        for(int c=min_c; c<=min_c+local_window_cols; c++) {
    //            int local_r = r-min_r;
    //            int local_c = c-min_c;
    //            satellite_patch.at<cv::Vec3b>(local_r, local_c) = satellite_img.at<cv::Vec3b>(r, c);
    //            satellite_mask_patch.at<cv::Vec3b>(local_r, local_c) = satellite_mask_road.at<cv::Vec3b>(r, c);
    //        }
    //    }

    cv::namedWindow("satellite_patch", CV_WINDOW_NORMAL);
    cv::imshow("satellite_patch", satellite_patch);
    cv::namedWindow("satellite_mask_patch", CV_WINDOW_NORMAL);
    cv::imshow("satellite_mask_patch", satellite_mask_patch);
    char patch_output_name[200], mask_output_name[200];
    sprintf(patch_output_name, "./Satellite_Terrain_data/KITTI/08/Satellite_Patch/satellite_patch_%06d.png", ID);
    sprintf(mask_output_name, "./Satellite_Terrain_data/KITTI/08/Satellite_Mask_Road/satellite_road_%06d.png", ID);
    cv::imwrite(patch_output_name, satellite_patch);
    cv::imwrite(mask_output_name, satellite_mask_patch);
    ID++;
    cv::waitKey(2);
}

void Satellite_Patch::Run2(double2D blh_pos, Eigen::Matrix3d R_w_l)
{
    // add error!
    //    double2D current_xy = blh2xy(blh_pos);
    //    double error = 5;
    //    double2D current_xy_with_error;
    //    current_xy_with_error.x = current_xy.x - error;
    //    current_xy_with_error.y = current_xy.y - error;
    //    double2D current_blh_with_error = xy2blh(current_xy_with_error);
    //    double2D pixel_pos_center = blhToPixelCoord(current_blh_with_error);

    // local window pixel
    //    经纬度转像素
    double2D pixel_pos_center = blhToPixelCoord(blh_pos);
    std::cout << "  over blh2pixel" << std::endl;

    //    1.4045  1.5281
    //    double resolution = 0.297725;  // KITTI: 0.297725
    //    double resolution_rows = 1.4045;
    //    double resolution_cols = 1.5281;
    //      单个像素分辨率
    double resolution_rows = 0.265;
    double resolution_cols = 0.265;

    int local_window_rows = ceil(40 / resolution_rows);
    int local_window_cols = ceil(40 / resolution_cols);
    // ALS: (122, 154)
    // KITTI: (135, 135)
    // school: (29, 27)
    if (local_window_rows != 151 || local_window_cols != 151)
        std::cout << "error size: " << local_window_rows << " ," << local_window_cols << std::endl;

    //    前后左右各40m，加当前所在像素
    int whole_rows = 2 * local_window_rows + 1;
    int whole_cols = 2 * local_window_cols + 1;

    cv::Mat satellite_patch = cv::Mat::zeros(whole_rows, whole_cols, CV_8UC3);
    cv::Mat satellite_mask_patch = cv::Mat::zeros(whole_rows, whole_cols, CV_8UC3);

    //    在大卫星图上切割当前点周边80*80的区域
    int min_r = pixel_pos_center.y - local_window_rows;
    int max_r = pixel_pos_center.y + local_window_rows;
    int min_c = pixel_pos_center.x - local_window_cols;
    int max_c = pixel_pos_center.x + local_window_cols;

    int center_idx = whole_rows / 2 + 1;

    std::cout << "  start for loop" << std::endl;

    //    整体仍然是局部转全局，不会留下空洞
    for (int r = 0; r < whole_rows; r++)
    {
        for (int c = 0; c < whole_cols; c++)
        {

            double global_x = (c - center_idx) * resolution_cols;
            double global_y = (r - center_idx) * resolution_rows;

            // world ->local
            Eigen::Vector3d global_grid, local_grid;
            global_grid << global_x, global_y, 1;
            local_grid = R_w_l.inverse() * global_grid;

            // local -> grid
            double local_c = local_grid(0) / resolution_cols + center_idx;
            double local_r = local_grid(1) / resolution_rows + center_idx;

            int global_r = round(local_r + min_r);
            int global_c = round(local_c + min_c);
            satellite_patch.at<cv::Vec3b>(r, c) = satellite_img.at<cv::Vec3b>(global_r, global_c);
            satellite_mask_patch.at<cv::Vec3b>(r, c) = satellite_mask_road.at<cv::Vec3b>(global_r, global_c);
        }
    }
    double2D pixel_tl = blhToPixelCoord(top_left_blh);
    double2D pixel_dr = blhToPixelCoord(low_right_blh);

    cv::circle(satellite_mask_road_cp, cv::Point(pixel_tl.x, pixel_tl.y), 1.5, cv::Vec3b(0, 255, 255), -1);
    cv::circle(satellite_mask_road_cp, cv::Point(pixel_dr.x, pixel_dr.y), 1.5, cv::Vec3b(255, 255, 0), -1);
    cv::circle(satellite_mask_road_cp, cv::Point(pixel_pos_center.x, pixel_pos_center.y), 1.5, cv::Vec3b(0, 255, 0), -1);

    cv::namedWindow("whole mask img", CV_WINDOW_NORMAL);
    cv::imshow("whole mask img", satellite_mask_road_cp);

    //    for(int r=min_r; r<=min_r+local_window_rows; r++) {
    //        for(int c=min_c; c<=min_c+local_window_cols; c++) {
    //            int local_r = r-min_r;
    //            int local_c = c-min_c;
    //            satellite_patch.at<cv::Vec3b>(local_r, local_c) = satellite_img.at<cv::Vec3b>(r, c);
    //            satellite_mask_patch.at<cv::Vec3b>(local_r, local_c) = satellite_mask_road.at<cv::Vec3b>(r, c);
    //        }
    //    }

    std::cout << "  start save satellite_patch" << std::endl;

    cv::namedWindow("satellite_patch", CV_WINDOW_NORMAL);
    cv::imshow("satellite_patch", satellite_patch);
    cv::namedWindow("satellite_mask_patch", CV_WINDOW_NORMAL);
    cv::imshow("satellite_mask_patch", satellite_mask_patch);
    char patch_output_name[500], mask_output_name[500];

    sprintf(patch_output_name, "%s/satellite_patch_%06d.png", str_patch_output_name.c_str(), ID);
    sprintf(mask_output_name, "%s/satellite_road_%06d.png", str_mask_output_name.c_str(), ID);
    std::cout << mask_output_name << std::endl;
    cv::imwrite(patch_output_name, satellite_patch);
    cv::imwrite(mask_output_name, satellite_mask_patch);
    ID++;
    cv::waitKey(2);
}

void Satellite_Patch::Run3(double2D blh_pos, Eigen::Matrix3d R_w_l, cv::Mat &satellite_mask_patch)
{
    // add error!
    //    double2D current_xy = blh2xy(blh_pos);
    //    double error = 5;
    //    double2D current_xy_with_error;
    //    current_xy_with_error.x = current_xy.x - error;
    //    current_xy_with_error.y = current_xy.y - error;
    //    double2D current_blh_with_error = xy2blh(current_xy_with_error);
    //    double2D pixel_pos_center = blhToPixelCoord(current_blh_with_error);

    // local window pixel
    //    经纬度转像素
    double2D pixel_pos_center = blhToPixelCoord(blh_pos);
    // std::cout << "  over blh2pixel" << std::endl;

    //    1.4045  1.5281
    //    double resolution = 0.297725;  // KITTI: 0.297725
    //    double resolution_rows = 1.4045;
    //    double resolution_cols = 1.5281;

    //      单个像素分辨率
    double resolution_rows = 0.265;
    double resolution_cols = 0.265;

    int local_window_rows = ceil(40 / resolution_rows);
    int local_window_cols = ceil(40 / resolution_cols);

    // ALS: (122, 154)
    // KITTI: (135, 135)
    // school: (29, 27)

    if (local_window_rows != 151 || local_window_cols != 151)
        std::cout << "error size: " << local_window_rows << " ," << local_window_cols << std::endl;

    //    前后左右各40m，加当前所在像素
    int whole_rows = 2 * local_window_rows + 1;
    int whole_cols = 2 * local_window_cols + 1;

    cv::Mat satellite_patch = cv::Mat::zeros(whole_rows, whole_cols, CV_8UC3);
    satellite_mask_patch = cv::Mat::zeros(whole_rows, whole_cols, CV_8UC3);

    //    在大卫星图上切割当前点周边80*80的区域
    int min_r = pixel_pos_center.y - local_window_rows;
    int max_r = pixel_pos_center.y + local_window_rows;
    int min_c = pixel_pos_center.x - local_window_cols;
    int max_c = pixel_pos_center.x + local_window_cols;

    int center_idx = whole_rows / 2 + 1;

    // std::cout << "  start for loop" << std::endl;

    //    整体仍然是局部转全局，不会留下空洞
    for (int r = 0; r < whole_rows; r++)
    {
        for (int c = 0; c < whole_cols; c++)
        {

            double global_x = (c - center_idx) * resolution_cols;
            double global_y = (r - center_idx) * resolution_rows;

            // world ->local
            Eigen::Vector3d global_grid, local_grid;
            global_grid << global_x, global_y, 1;
            local_grid = R_w_l.inverse() * global_grid;

            // local -> grid
            double local_c = local_grid(0) / resolution_cols + center_idx;
            double local_r = local_grid(1) / resolution_rows + center_idx;

            int global_r = round(local_r + min_r);
            int global_c = round(local_c + min_c);
            satellite_patch.at<cv::Vec3b>(r, c) = satellite_img.at<cv::Vec3b>(global_r, global_c);
            satellite_mask_patch.at<cv::Vec3b>(r, c) = satellite_mask_road.at<cv::Vec3b>(global_r, global_c);
        }
    }

    double2D pixel_tl = blhToPixelCoord(top_left_blh);
    double2D pixel_dr = blhToPixelCoord(low_right_blh);

    // cv::circle(satellite_mask_road_cp, cv::Point(pixel_tl.x, pixel_tl.y), 1.5, cv::Vec3b(0, 255, 255), -1);
    // cv::circle(satellite_mask_road_cp, cv::Point(pixel_dr.x, pixel_dr.y), 1.5, cv::Vec3b(255, 255, 0), -1);
    // cv::circle(satellite_mask_road_cp, cv::Point(pixel_pos_center.x, pixel_pos_center.y), 1.5, cv::Vec3b(0, 255, 0), -1);

    // cv::namedWindow("whole mask img", CV_WINDOW_NORMAL);
    // cv::imshow("whole mask img", satellite_mask_road_cp);

    cv::namedWindow("satellite_patch", CV_WINDOW_NORMAL);
    cv::imshow("satellite_patch", satellite_patch);
    cv::namedWindow("satellite_mask_patch", CV_WINDOW_NORMAL);
    cv::imshow("satellite_mask_patch", satellite_mask_patch);

    // char patch_output_name[500], mask_output_name[500];
    // sprintf(patch_output_name, "%s/satellite_patch_%06d.png", str_patch_output_name.c_str(),ID);
    // sprintf(mask_output_name, "%s/satellite_road_%06d.png", str_mask_output_name.c_str(), ID);
    // std::cout<<mask_output_name<<std::endl;
    // cv::imwrite(patch_output_name, satellite_patch);
    // cv::imwrite(mask_output_name, satellite_mask_patch);
    // ID++;

    cv::waitKey(2);
}