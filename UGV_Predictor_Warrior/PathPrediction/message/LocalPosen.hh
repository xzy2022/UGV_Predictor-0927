/*
LocalPosen.hh

This C++ header file defines the NML Messages for LOCALPOSE
Template Version 1.1

MODIFICATIONS:
Thu Apr 25 09:55:53 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef LOCALPOSEN_HH
#define LOCALPOSEN_HH

// Include Files
//#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

// Define the integer type ids.
#define LOCALPOSE_MSG_TYPE 145000
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

#define LOCALPOSE_DATA_SIZE 50

// Define the NML Message Classes
class LOCALPOSE_MSG : public NMLmsgEx
{
public:

	//Constructor
    LOCALPOSE_MSG(){};

	// CMS Update Function
//    void update(CMS *){};

	// Place custom variables here.
//    LOCAL_POS_DATA  data;
//    LOCAL_POS_DATA  LocalPoseData[LOCALPOSE_DATA_SIZE];       // LocalPose array

//    INT32           DataNumber;                 // effective elements of Data
//    double          SystemTimeDiff;             // LocalPose的时间与本计算机系统时间的差别

//    LOCAL_POS_DATA  to_delete2[LOCALPOSE_DATA_SIZE];       // LocalPose array

//    INT32           to_delete3;                 // effective elements of Data
//    double          to_delete4;
};

// Declare NML format function
//extern int LocalPoseFormat(NMLTYPE, void *, CMS *);

#endif 	// LOCALPOSEN_HH
