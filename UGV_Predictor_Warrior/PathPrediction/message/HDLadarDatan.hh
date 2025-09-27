/*
HDLadarDatan.hh

This C++ header file defines the NML Messages for HDLadarData
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:05 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef HDLADARDATAN_HH
#define HDLADARDATAN_HH

// Include Files
//#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"
#include "LocalPosen.hh"




// Define the integer type ids.
#define HDLADARDATA_MSG_TYPE 11003
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
}PointCoordinate64;

typedef struct
{
    short x;
    short y;
    short z;
    unsigned char pLabel;
}PointLabel64;

// Define the NML Message Classes
class HDLADARDATA_MSG : public NMLmsgEx
{
public:
    //Constructor
    HDLADARDATA_MSG(){};

    // CMS Update Function
//    void update(CMS *);

    // Place custom variables here.

    PointCoordinate64 HDData[PACKETNUM64*6][64];

    UINT32 packetNum;
    UINT32 FrameEndTime;
};

// Declare NML format function
//extern int HDLadarDataFormat(NMLTYPE, void *, CMS *);

#endif 	// HDLADARDATAN_HH
