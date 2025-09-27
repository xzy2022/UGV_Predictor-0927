#ifndef POSE_QUEUE_INTERFACE_H_
#define POSE_QUEUE_INTERFACE_H_

#include <map>
#include <stdint.h>

#include "GlobalPositionInfon.hh"

#include "PoseQueue.h"

class PoseQueueInterface {

public:

//    PoseQueueInterface(NML *_GLOBALPOSITION_CHANNEL, UINT32 queue_size = 400, bool extrapolate_data = true, double extrapolation_time_threshold = 30.0);
    PoseQueueInterface(UINT32 queue_size = 400, bool extrapolate_data = true, double extrapolation_time_threshold = 50.0);


    PoseQueue *pose_queue;

//    NML *GLOBALPOSITION_CHANNEL;
    GLOBALPOSITIONINFO_MSG *GlobalPositionInfo_data;

    void Start();
//    void StartReceiver();

    void push(const Pose& pose, double timestamp);

    void poseAt(double time, Pose &res_pose);
    void latestPosition(PositionData &position);

    pthread_mutex_t m_Mutex;
};


#endif
