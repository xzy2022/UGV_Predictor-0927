/*
HDLadarDatan.hh

This C++ header file defines the NML Messages for HDLadarData
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:05 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef RS128DATAN_HH
#define RS128DATAN_HH

// Include Files
//#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"
#include "LocalPosen.hh"


// Define the integer type ids.
#define LIDARRS128DATA_MSG_TYPE 11028
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.
// 64线雷达点，单位厘米
typedef struct
{
    short x;      //车体坐标系,
    short y;
    short z;
//    int angleH;   //角度，单位为0.01度
//    int angleV;
//    short realDistance; // 真实距离，单位厘米
    unsigned char Intensity;
}PointCoordinate128;

typedef struct
{
    double x;
    double y;
    double z;
    int pLabel;
}PointLabel128;

// Define the NML Message Classes
class LIDARRS128DATA_MSG : public NMLmsgEx
{
public:
    //Constructor
    LIDARRS128DATA_MSG();

    // CMS Update Function
//    void update(CMS *);

    // Place custom variables here.

    PointCoordinate128 HDData[PACKETNUM64*3][128];

    UINT32 packetNum;
    UINT32 FrameEndTime;
};

// Declare NML format function
//extern int LIDARRS128DataFormat(NMLTYPE, void *, CMS *);

#endif 	// HDLADARDATAN_HH
