
#include "CoordTransformation.h"

void Ang2R_3D_Our(Eigen::Vector3f &angle, Eigen::Matrix3f &R)
{
//    first rotate around z axis (yaw angle)
//    then rotate around x axis (pitch angle)
//    lastly rotate around y axis (roll algle)
//    angle[0] = roll; angle[1] = pitch; angle[2] = yaw
//    R = Rot_z(yaw) * Rot_x(pitch) * Rot_y(roll)

    float sr,cr,sp,cp,sy,cy;
    sr = sin(angle(0));
    cr = cos(angle(0));
    sp = sin(angle(1));
    cp = cos(angle(1));
    sy = sin(angle(2));
    cy = cos(angle(2));

    R(0,0) = cy*cr - sy*sp*sr;
    R(0,1) = -sy*cp;
    R(0,2) = cy*sr + sy*sp*cr;

    R(1,0) = sy*cr + cy*sp*sr;
    R(1,1) = cy*cp;
    R(1,2) = sy*sr - cy*sp*cr;

    R(2,0) = -cp*sr;
    R(2,1) = sp;
    R(2,2) = cp*cr;
}

Eigen::Matrix3d YPR2RotationMatrix(double3D ypr)
{

    Eigen::Matrix3d Rx,Ry,Rz,R_w_l;
    double crz = cos(ypr.x);
    double srz = sin(ypr.x);
    double crx = cos(ypr.y);
    double srx = sin(ypr.y);
    double cry = cos(ypr.z);
    double sry = sin(ypr.z);
    Ry<<cry, 0, sry,
            0 ,1, 0,
            -sry, 0, cry;
    Rx<<1, 0, 0,
            0, crx, -srx,
            0, srx, crx;
    Rz<<crz, -srz, 0,
            srz, crz, 0,
            0 , 0, 1;
    R_w_l = Rz*Rx*Ry; // R_azimuth * R_pitch * R_roll

    return R_w_l;
}


void R2Ang_3D_Our(Eigen::Matrix3d &R, Eigen::Vector3d &angle)
{
// Z1X2Y3, applicable for our coordinate (X->right, Y->forward, Z->upward)
//    angle[0] = yaw; angle[1] = pitch; angle[2] = roll
    angle(1) = asin(R(2,1));
    double cp = cos(angle(1));
    angle(2) = -atan2((R(2,0)/cp),(R(2,2)/cp));
    angle(0) = -atan2((R(0,1)/cp),(R(1,1)/cp));
}

void R2Ang_3D_Lidar(Eigen::Matrix3f &R, Eigen::Vector3f &angle)
{
// Z1Y2X3, applicable for lidar coordinate (X->forward, Y->left, Z->upward)
//    angle[0] = yaw; angle[1] = pitch; angle[2] = roll
    angle(1) = -asin(R(2,0));
    float cp = cos(angle(1));
    angle(2) = atan2((R(2,1)/cp),(R(2,2)/cp));
    angle(0) = atan2((R(1,0)/cp),(R(0,0)/cp));
}

void R2Ang_3D_Loam(Eigen::Matrix3f &R, Eigen::Vector3f &angle)
{
// Y1X2Z3, applicable for loam coordinate (X->left, Y->upward, Z->forward)
//    angle[0] = yaw; angle[1] = pitch; angle[2] = roll
    angle(1) = -asin(R(1,2));
    float cp = cos(angle(1));
    angle(2) = atan2((R(1,0)/cp),(R(1,1)/cp));
    angle(0) = atan2((R(0,2)/cp),(R(2,2)/cp));
}

void NormalTr2loamtransform(Eigen::Matrix4d Tr, double* transform_)
{
    // transform_: Tr_BA in loam coordinate (left-up-forward)
    // Tr: Tr_AB in our coordinate (right-forward-up)

    Eigen::Vector3d ang;
    Eigen::Matrix3d R = Tr.topLeftCorner<3,3>();
    R2Ang_3D_Our(R, ang);

    transform_[0] = ang(1); // -(-pitch)
    transform_[1] = -ang(0); // -yaw
    transform_[2] = -ang(2); // -roll

    Eigen::Vector3d T_AB_LOAM, T_BA_LOAM;
    T_AB_LOAM[0] = -Tr(0,3);
    T_AB_LOAM[1] = Tr(2,3);
    T_AB_LOAM[2] = Tr(1,3);

    double cry = cos(-transform_[1]);
    double sry = sin(-transform_[1]);
    double crx = cos(-transform_[0]);
    double srx = sin(-transform_[0]);
    double crz = cos(-transform_[2]);
    double srz = sin(-transform_[2]);

    Eigen::Matrix3d Rx,Ry,Rz,R_AB;
    Ry<<cry, 0, sry,
            0 ,1, 0,
            -sry, 0, cry;
    Rx<<1, 0, 0,
            0, crx, -srx,
            0, srx, crx;
    Rz<<crz, -srz, 0,
            srz, crz, 0,
            0 , 0, 1;
    R_AB = Ry*Rx*Rz;

    T_BA_LOAM = -R_AB.transpose() * T_AB_LOAM;    // T_BA =  -R_BA * T_AB

    transform_[3] = T_BA_LOAM[0];
    transform_[4] = T_BA_LOAM[1];
    transform_[5] = T_BA_LOAM[2];
}


void loamtransform2NormalTr(float *transform_, Eigen::Matrix4f &Tr)
{
    // transform_: Tr_BA in loam coordinate (left-up-forward)
    // Tr: Tr_AB in our coordinate (right-forward-up)

    double cry = cos(-transform_[1]);
    double sry = sin(-transform_[1]);
    double crx = cos(-transform_[0]);
    double srx = sin(-transform_[0]);
    double crz = cos(-transform_[2]);
    double srz = sin(-transform_[2]);

    Eigen::Matrix3f Rx,Ry,Rz,R_AB,R;
    Ry<<cry, 0, sry,
            0 ,1, 0,
            -sry, 0, cry;
    Rx<<1, 0, 0,
            0, crx, -srx,
            0, srx, crx;
    Rz<<crz, -srz, 0,
            srz, crz, 0,
            0 , 0, 1;
    R_AB = Ry*Rx*Rz;

    Eigen::Vector3f T_BA,T_AB_LOAM;

    T_BA<<transform_[3],transform_[4],transform_[5];
    T_AB_LOAM = -R_AB*T_BA;

    double c_a = cos(-transform_[1]);  // aizmuth-->rot z
    double s_a = sin(-transform_[1]);
    double c_p = cos(transform_[0]);  // pitch---> rot x
    double s_p = sin(transform_[0]);
    double c_r = cos(-transform_[2]);  // roll---> rot y
    double s_r = sin(-transform_[2]);
    Ry<<c_r, 0, s_r,
            0 ,1, 0,
            -s_r, 0, c_r;
    Rx<<1, 0, 0,
            0, c_p, -s_p,
            0, s_p, c_p;
    Rz<<c_a, -s_a, 0,
            s_a, c_a, 0,
            0 , 0, 1;
    R = Rz*Rx*Ry;  // firstly azimuth, next pitch, finally roll

    Tr.setZero();
    Tr.topLeftCorner<3,3>() = R;
    Tr(0,3) = -T_AB_LOAM[0];
    Tr(1,3) = T_AB_LOAM[2];
    Tr(2,3) = T_AB_LOAM[1];
    Tr(3,3) = 1;
}




void predictLoamTransformFromIns(int this_x, int this_y, int this_z, int this_azimuth, int this_pitch, int this_roll,
                             int last_x, int last_y, int last_z, int last_azimuth, int last_pitch, int last_roll, float *transform_)
{
    // loam coordinate: X-left, Y-upward, Z-forward
    // Denote last is A, this (current) is B
    // x,y,z: cm (in global coordinate)
    // azimuth, pitch, roll: 0.01 degree (in global coordinate)
    // x axis point to east, y axis point to north, z is upwards
    // when the vehicle faces east, azimuth is 0; when the vehicle faces north, the vehicle local coordinate (right-forward-up) is the same as the global coordinate, but the azimuth is 9000

    // transform stores Tr_BA = Tr_this_last

    Eigen::Matrix3d Rx,Ry,Rz,R_w_B,R_w_A,R_A_B;
    double cry = cos((this_azimuth-9000)*PI/18000.0f);
    double sry = sin((this_azimuth-9000)*PI/18000.0f);
    double crx = cos(-this_pitch*PI/18000.0f);
    double srx = sin(-this_pitch*PI/18000.0f);
    double crz = cos(this_roll*PI/18000.0f);
    double srz = sin(this_roll*PI/18000.0f);
    Ry<<cry, 0, sry,
            0 ,1, 0,
            -sry, 0, cry;
    Rx<<1, 0, 0,
            0, crx, -srx,
            0, srx, crx;
    Rz<<crz, -srz, 0,
            srz, crz, 0,
            0 , 0, 1;
    R_w_B = Ry*Rx*Rz;

    cry = cos((last_azimuth-9000)*PI/18000);
    sry = sin((last_azimuth-9000)*PI/18000);
    crx = cos(-last_pitch*PI/18000);
    srx = sin(-last_pitch*PI/18000);
    crz = cos(last_roll*PI/18000);
    srz = sin(last_roll*PI/18000);
    Ry<<cry, 0, sry,
            0 ,1, 0,
            -sry, 0, cry;
    Rx<<1, 0, 0,
            0, crx, -srx,
            0, srx, crx;
    Rz<<crz, -srz, 0,
            srz, crz, 0,
            0 , 0, 1;
    R_w_A = Ry*Rx*Rz;

    R_A_B = R_w_A.transpose() * R_w_B;
    float rx,ry,rz;
    rx = -asin(R_A_B(1,2));
    ry = atan2(R_A_B(0,2)/cos(rx),R_A_B(2,2)/cos(rx));
    rz = atan2(R_A_B(1,0)/cos(rx),R_A_B(1,1)/cos(rx));
    transform_[0] = -rx;
    transform_[1] = -ry;
    transform_[2] = -rz;

    Eigen::Vector3d t_l_B_A,t_w_B_A;
    double delta_x,delta_y,delta_z;
    delta_z = (last_y - this_y)/100.0f;
    delta_y = (last_z - this_z)/100.0f;
    delta_x = -(last_x - this_x)/100.0f;
    t_w_B_A<<delta_x, delta_y, delta_z;
    t_l_B_A = R_w_B.transpose() * t_w_B_A;

    transform_[3] = t_l_B_A(0);
    transform_[4] = t_l_B_A(1);
    transform_[5] = t_l_B_A(2);
}


Eigen::Matrix4d transformPose2Tr(const Pose &A)
{
    // pose.azimuth, pose.pitch, pose.roll: 0.01 degree
    // pose.x, pose.y, pose.z: cm
    // Tr(0,3), Tr(1,3), Tr(2,3): cm

        Eigen::Matrix3d Rx,Ry,Rz,R_w_l;
        double crz = cos((A.azimuth-9000)*PI/18000.0f);
        double srz = sin((A.azimuth-9000)*PI/18000.0f);
        double crx = cos(A.pitch*PI/18000.0f);
        double srx = sin(A.pitch*PI/18000.0f);
        double cry = cos(A.roll*PI/18000.0f);
        double sry = sin(A.roll*PI/18000.0f);
        Ry<<cry, 0, sry,
                0 ,1, 0,
                -sry, 0, cry;
        Rx<<1, 0, 0,
                0, crx, -srx,
                0, srx, crx;
        Rz<<crz, -srz, 0,
                srz, crz, 0,
                0 , 0, 1;
        R_w_l = Rz*Rx*Ry; // R_azimuth * R_pitch * R_roll

        Eigen::Matrix4d Tr_w_l;
        Tr_w_l.setZero();
        Tr_w_l.topLeftCorner<3,3>() = R_w_l;
//        Tr_w_l(0,3) = A.x / 100.0f;
//        Tr_w_l(1,3) = A.y / 100.0f;
//        Tr_w_l(2,3) = A.z / 100.0f;
        Tr_w_l(0,3) = A.x;      // modified on 20190306
        Tr_w_l(1,3) = A.y;
        Tr_w_l(2,3) = A.z;
        Tr_w_l(3,3) = 1;

        return Tr_w_l;
}


Eigen::Matrix3d angle2R(double azimuth, double pitch, double roll)
{
    // azimuth, pitch, roll: radian

    Eigen::Matrix3d delta_R, R_z, R_x, R_y;
    double cz = cos(azimuth);
    double sz = sin(azimuth);
    double cx = cos(pitch);
    double sx = sin(pitch);
    double cy = cos(roll);
    double sy = sin(roll);
    R_z << cz, -sz, 0,
            sz, cz, 0,
            0, 0, 1;
    R_x << 1, 0, 0,
            0, cx, -sx,
            0, sx, cx;
    R_y << cy, 0, sy,
            0, 1, 0,
            -sy, 0, cy;
    delta_R = R_z * R_x* R_y;
    return delta_R;
}


void transformGlobal2Local(const Pose2D &A, double global_x, double global_y, double *local_x, double *local_y)
{
    // pose.azimuth, pose.pitch, pose.roll: 0.01 degree
    // pose.x, pose.y, pose.z: cm

    // global_x, global_y: cm

//    Eigen::Matrix3d Rz,R_w_l;
    double crz = cos((A.azimuth-9000)*PI/18000.0f);
    double srz = sin((A.azimuth-9000)*PI/18000.0f);
//    Rz<<crz, -srz, 0,
//            srz, crz, 0,
//            0 , 0, 1;
//    R_w_l = Rz; // R_azimuth

    // [global_x; global_y] = R_w_l * [local_x; local_y] + [A.x; A.y]
    // [local_x; local_y]  = R_w_l' * ([global_x; global_y] - [A.x; A.y]) = [crz srz; -srz crz] * ([global_x; global_y] - [A.x; A.y])

    *local_x = crz * (global_x - A.x) + srz * (global_y - A.y);
    *local_y = -srz * (global_x - A.x) + crz * (global_y - A.y);
}

void transformLocal2Global(const Pose2D &A, double local_x, double local_y, double *global_x, double *global_y)
{
    // pose.azimuth, pose.pitch, pose.roll: 0.01 degree
    // pose.x, pose.y, pose.z: cm

    // global_x, global_y: cm

//    Eigen::Matrix3d Rz,R_w_l;
    double crz = cos((A.azimuth-9000)*PI/18000.0f);
    double srz = sin((A.azimuth-9000)*PI/18000.0f);
//    Rz<<crz, -srz, 0,
//            srz, crz, 0,
//            0 , 0, 1;
//    R_w_l = Rz; // R_azimuth

    // [global_x; global_y] = R_w_l * [local_x; local_y] + [A.x; A.y] = [crz -srz; srz crz] * [local_x; local_y] + [A.x; A.y]
    // [local_x; local_y]  = R_w_l' * ([global_x; global_y] - [A.x; A.y]) = [crz srz; -srz crz] * ([global_x; global_y] - [A.x; A.y])

    *global_x = crz * local_x - srz * local_y + A.x;
    *global_y = srz * local_x + crz * local_y + A.y;
}



CoordTransformation2D::CoordTransformation2D()
{

}

void CoordTransformation2D::setPose(Pose2D pose)
{
    x_ = pose.x;
    y_ = pose.y;
    crz_ = cos((pose.azimuth-9000)*PI/18000.0f);
    srz_ = sin((pose.azimuth-9000)*PI/18000.0f);
}

void CoordTransformation2D::transformGlobal2Local(double global_x, double global_y, double *local_x, double *local_y)
{
    *local_x = crz_ * (global_x - x_) + srz_ * (global_y - y_);
    *local_y = -srz_ * (global_x - x_) + crz_ * (global_y - y_);
}

void CoordTransformation2D::transformLocal2Global(double local_x, double local_y, double *global_x, double *global_y)
{
    *global_x = crz_ * local_x - srz_ * local_y + x_;
    *global_y = srz_ * local_x + crz_ * local_y + y_;
}


