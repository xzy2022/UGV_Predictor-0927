
#ifndef COMMONDEFINITION_HH
#define COMMONDEFINITION_HH


#ifndef __INT64__
#define __INT64__
typedef  signed     long        INT64;
#endif

#ifndef __UINT64__
#define __UINT64__
typedef  unsigned   long        UINT64;
#endif

#ifndef __INT32__
#define __INT32__
typedef  signed     int         INT32;
#endif

#ifndef __UINT32__
#define __UINT32__
typedef  unsigned   int         UINT32;
#endif

#ifndef __INT16__
#define __INT16__
typedef  signed     short       INT16;
#endif

#ifndef __UINT16__
#define __UINT16__
typedef  unsigned   short       UINT16;
#endif

#ifndef __INT8__
#define __INT8__
typedef  signed     char        INT8;
#endif

#ifndef __UINT8__
#define __UINT8__
typedef  unsigned   char        UINT8;
#endif


#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

//#ifndef Pi
//#define Pi 3.1415926535897932384626433832795
//#endif



#define SC_ANG              (100.0)
#define SC_H                (100.0)
#define SC_POS              (10000000.0)
#define SC_T                (1000.0)
#define SC_TPVEL            (1000.0)

#define ENHANCED_IMAGE_SIZE 2000*1000
#define COLORIMAGE_SIZE 2000*1000

#define MAX_VEHICLE_OBJ 30
#define MAX_PEDESTRIAN_OBJ 30
#define MAX_ROAD_OBJ 5
#define MAX_INTERSECTION_OBJ 3

#define MAX_LANE_OBJ 10

#define MAX_IRROAD_OBJ 5



#define GRID_SIZE 20


//#define LOCALMAP_WIDTH      800
//#define LOCALMAP_HEIGHT     800
//#define LOCALMAP_VEHICLEX   400
//#define LOCALMAP_VEHICLEY   400
//#define LOCALMAP_WIDTH      350
//#define LOCALMAP_HEIGHT     575
//#define LOCALMAP_VEHICLEX   175
//#define LOCALMAP_VEHICLEY   400

#define LOCALMAP_WIDTH      250
#define LOCALMAP_HEIGHT     500
#define LOCALMAP_VEHICLEX   125
#define LOCALMAP_VEHICLEY   400
#define LOCALMAP_SIZE    LOCALMAP_WIDTH*LOCALMAP_HEIGHT


// AttributeMap: 局部坐标系，前方80米，后方20米，左右25米
#define ATTRIBUTEMAP_WIDTH   LOCALMAP_WIDTH
#define ATTRIBUTEMAP_HEIGHT  LOCALMAP_HEIGHT
#define ATTRIBUTEMAP_VEHICLEX       LOCALMAP_VEHICLEX
#define ATTRIBUTEMAP_VEHICLEY       LOCALMAP_VEHICLEY
#define LOCAL_ATTRIBUTEMAP_SIZE LOCALMAP_SIZE


#define IR_IMAGE_SIZE 640*480
#define IR_IMAGE_SIZE_BAYER 1296*964
#define IR_IMGAE_SIZE_NIR 1296*966

//#define PACKETNUM 600
#define PACKETNUM64 650
#define PACKETNUM128 2000

#define PACKETNUM_LADAR32 200

#define PACKETNUM40 400

#define PACKETNUM16 400

#define PACKETNUMRS_BP 160
///////////////////////////////////////
#define GLOBALPOINTNUM  2000

#define PACKETNUMHS64 5000

//definition for local path planning
#define MAXPLANPOINTS 50
#define MYPLANPOINTS 100

#define MAX_SCAN_DATA_NUM 1800

// definition by sun zhenping
const int  DIRECT_ACTUATOR=0,REMOTE_PILOT=1,AUTO_PILOT=2;
#define CMD_SOURCE int

const int      OFF=0,ON=1;
#define LOGIC_STATE int

const int      PARK=0,BACKWARD=1,NEURAL=2,FORWARD=3;
#define TRANS_POSITION int

#undef TRANS_POSITION
const int L_POSITION=2,L2_POSITION=3,D_POSITION=4,N_POSITION=5,R_POSITION=6,P_POSITION=7;
#define TRANS_POSITION int

#define RangeImageHeight 128
#define RangeImageWidth 1800


enum UGV_MODE_CMD
{
	UGV_MODE_CMD_AU=0,
    UGV_MODE_CMD_KP=1,
    UGV_MODE_CMD_DR=2,
    UGV_MODE_CMD_GC=3
};

// end of definition by sun zhenping

enum COMMAND_TYPE
{
	CMD_START_RECORDING,
	CMD_STOP_RECORDING,
	CMD_EXIT_RECORDING,
	//add other commands here:
	CMD_START_PROCESS,
	CMD_EXIT_PROCESS
};

enum LC_STATE
{
    FLAMEOUT=100, 
    INFLAME, 
    ADVANCE, 
    BACKOFF, 
    VEHICLEUNKNOWN
};

enum AVT_IMAGE_WRITE_TYPE
{
    WRITE_ALL = 0,
    WRITE_LEFT = 1,
    WRITE_RIGHT = 2
};

enum POSITIVE_MAP_TYPE
{
    UNKNOWN_AREA     =0,
    GROUND_AREA      =1,
    OBSTACLE_AREA    =2,
    NEGTIVE_AREA     =3,
    HANGING_AREA     =4,
    CLIFF_AREA       =5,
    WATER_AREA       =6,
};


//
enum OBJECT_CLASSIFICATION_TYPE
{
    UNKNOWN_CLASS     =0,
    VEHICLE_CLASS     =1,
    PEDESTRIAN_CLASS  =2,
    BICYCLE_CLASS     =3,
    SMALL_OBJECT_CLASS=4,
    BIG_OBJECT_CLASS  =5,
    STATIC_OBJECT_CLASS=6
};



#endif 	// COMMONDEFINITION_HH
