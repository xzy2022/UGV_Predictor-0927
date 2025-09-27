#ifndef GLOBAL_DEFINE_HPP
#define GLOBAL_DEFINE_HPP

// system
#include <stdio.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
//#include <opencv2/opencv.hpp>
#include <string>
#include <time.h>

// Eigen
#include <Eigen/Eigen>
#include <Eigen/Dense>


#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#define     BASE_X      19695752.27
#define		BASE_Y      3125228.07


using namespace std;

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;


struct state_pose {
    Eigen::Vector3d pos;
    Eigen::Quaterniond rot;

};

namespace my_file {
static bool get_filelist_from_dir(std::string _path, std::vector<std::string> &_files)
{
    DIR *dir;
    dir = opendir(_path.c_str());

    if (dir == NULL)
    {
        printf("d == NULL");
        return false;
    }
    struct dirent *ptr;
    std::vector<std::string> file;
    while ((ptr = readdir(dir)) != NULL)
    {
        if (ptr->d_name[0] == '.')
            continue;

        file.push_back(ptr->d_name);
    }
    closedir(dir);
//    sort(file.begin(), file.end());

    sort(file.begin(), file.end(), [](string a, string b){
        return stoi(a) < stoi(b);
    });

    // 打印排序后的文件名
    for (const auto& file_name : file) {
        std::cout << file_name << std::endl;
    }

    _files = file;

    return true;
}
static bool is_exists(const std::string &name) {
    std::ifstream f(name.c_str());
    return f.good();
}

}



#endif // GLOBAL_DEFINE_HPP
