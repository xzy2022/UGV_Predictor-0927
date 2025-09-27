

#ifndef POSE_H_
#define POSE_H_

#include "Eigen/Core"

#ifndef PI
#define PI 3.14159265358979
#endif

struct Pose{
    double x;       // cm
    double y;       // cm
    double z;       // cm
    double azimuth;     // 0.01 degree, 0 points to east
    double pitch;
    double roll;
    // for judging gps_status
    int positionStatus;
    int sol_status;
    double pvax_dev[2];

    Pose(){x = 0; y = 0; z = 0; azimuth = 0; pitch = 0; roll = 0;}
    Pose(double _x, double _y, double _z, double _azimuth, double _pitch, double _roll):x(_x), y(_y), z(_z), azimuth(_azimuth), pitch(_pitch), roll(_roll){}
    inline void reset() {
        x = 0; y = 0; z = 0; azimuth = 0; pitch = 0; roll = 0;
    }
};

struct Pose2D{
    double x;       // cm
    double y;       // cm
    double azimuth;     // 0.01 degree, 0 points to east

    Pose2D(){x = 0; y = 0; azimuth = 0;}
    Pose2D(double _x, double _y, double _azimuth):x(_x), y(_y), azimuth(_azimuth){}
    inline void reset() {
        x = 0; y = 0; azimuth = 0;
    }
};



#endif
