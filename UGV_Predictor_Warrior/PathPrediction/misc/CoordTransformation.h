



#ifndef COORDTRANSFORMATION_H
#define COORDTRANSFORMATION_H

#include "Eigen/Core"
#include "misc/Pose/Pose.h"
#include "misc/FH_commen_definition.h"

void Ang2R_3D_Our(Eigen::Vector3f &angle, Eigen::Matrix3f &R);
void R2Ang_3D_Our(Eigen::Matrix3d &R, Eigen::Vector3d &angle);
void R2Ang_3D_Lidar(Eigen::Matrix3f &R, Eigen::Vector3f &angle);
void R2Ang_3D_Loam(Eigen::Matrix3f &R, Eigen::Vector3f &angle);

void NormalTr2loamtransform(Eigen::Matrix4d Tr, double *transform_);
void loamtransform2NormalTr(float *transform_, Eigen::Matrix4f &Tr);
void predictLoamTransformFromIns(int this_x, int this_y, int this_z, int this_azimuth, int this_pitch, int this_roll,
                             int last_x, int last_y, int last_z, int last_azimuth, int last_pitch, int last_roll, float *transform_);


Eigen::Matrix3d YPR2RotationMatrix(double3D ypr);
Eigen::Matrix4d transformPose2Tr(const Pose &A); // cm
Eigen::Matrix3d angle2R(double azimuth, double pitch, double roll);

void transformGlobal2Local(const Pose2D &A, double global_x, double global_y, double *local_x, double *local_y);
void transformLocal2Global(const Pose2D &A, double local_x, double local_y, double *global_x, double *global_y);

class CoordTransformation2D
{

public:
    CoordTransformation2D();

    void setPose(Pose2D pose);
    void transformGlobal2Local(double global_x, double global_y, double *local_x, double *local_y);
    void transformLocal2Global(double local_x, double local_y, double *global_x, double *global_y);

    double x_,y_;
    double crz_;     // cos(azimuth)
    double srz_;     // sin(azimuth)
};









#endif
