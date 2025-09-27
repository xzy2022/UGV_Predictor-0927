/*
GlobalPositionInfon.hh

This C++ header file defines the NML Messages for GlobalPositionInfo
Template Version 1.1

MODIFICATIONS:
Mon Apr 09 16:09:48 CST 2013	Created by hutingbo.

*/

// Prevent Multiple Inclusion
#ifndef GLOBALPOSITIONINFO_HH
#define GLOBALPOSITIONINFO_HH

// Include Files
//#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.


// Define the integer type ids.
#define GLOBALPOSITIONINFO_MSG_TYPE 11001
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

#define SC_ANG              (100.0)
#define SC_H                (100.0)
#define SC_POS              (10000000.0)
#define SC_T                (1000.0)
#define SC_TPVEL            (1000.0)


// the Same enum with SPAN_STATUS defined in span.hh
typedef enum {
    INS_INACTIVE_CP=0,INS_ALIGNING_CP,INS_SOLUTION_NOT_GOOD_CP,INS_SOLUTION_GOOD_CP,INS_BAD_GPS_AGREEMENT_CP,INS_ALIGNMENT_COMPLETE_CP
} SPAN_STATUS_CP ;



#define GLOBALPOS_ARRAYSIZE 50

// Define the NML Message Classes
class GLOBALPOSITIONINFO_MSG : public NMLmsgEx
{
public:
	//Constructor
    GLOBALPOSITIONINFO_MSG(){};

	// CMS Update Function
//    void update(CMS *){};

//    INT32 MessageID;
   // INT32 MessageSeqNum;
   // Place custom variables here.
//    PositionData Position1;  //to be deleted Position1
//    PositionData positionData[GLOBALPOS_ARRAYSIZE];

    // RealGpsTime=RealSystemMillisecond+(TimeDiffDays%7)*24*60*60*1000-TimeSystemMillisecond+OriginGPSMillisecond
    // RealGpsTime=RealGpsTime%(7*24*60*60*1000)
//    INT32   DataNumber;
//    double  SystemTimeDiff;

//        INT32 to_delete5;
//        PositionData to_delete6;  //to be deleted Position1
//        PositionData to_delete7[GLOBALPOS_ARRAYSIZE];
//        INT32   to_delete8;
//        double  to_delete9;
};

// Declare NML format function
//extern int GlobalPositionInfoFormat(NMLTYPE, void *, CMS *);

#endif
