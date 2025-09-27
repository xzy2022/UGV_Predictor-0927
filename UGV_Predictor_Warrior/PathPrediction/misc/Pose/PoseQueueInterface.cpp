
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>

#include "PoseQueueInterface.h"

using std::string;


void *ReceiverRun(void *arg)
{
//    PoseQueueInterface *mydrs = (PoseQueueInterface *)arg;
//    mydrs->StartReceiver();
}

//PoseQueueInterface::PoseQueueInterface(NML *_GLOBALPOSITION_CHANNEL, UINT32 queue_size, bool extrapolate_data, double extrapolation_time_threshold)
//{
//    GLOBALPOSITION_CHANNEL = _GLOBALPOSITION_CHANNEL;
//    GlobalPositionInfo_data = (GLOBALPOSITIONINFO_MSG *) GLOBALPOSITION_CHANNEL->get_address();

//    pose_queue = new PoseQueue(queue_size, extrapolate_data, extrapolation_time_threshold);

//    pthread_mutex_init(&m_Mutex,NULL);
//}

PoseQueueInterface::PoseQueueInterface(UINT32 queue_size, bool extrapolate_data, double extrapolation_time_threshold)
{
    GlobalPositionInfo_data = new GLOBALPOSITIONINFO_MSG;
    pose_queue = new PoseQueue(queue_size, extrapolate_data, extrapolation_time_threshold);

    pthread_mutex_init(&m_Mutex,NULL);
}

void PoseQueueInterface::Start()
{
    pthread_t a_thread;
    int res = pthread_create(&a_thread,NULL,ReceiverRun,this);
    if(res !=0)
    {
        printf("receiver thread create failed!!!!!!!!!!!!!!!!!!!!!!!!\n");
        return;
    }
}

//void PoseQueueInterface::StartReceiver()
//{
//    while(1) {
//        NMLTYPE nml_read_ret = GLOBALPOSITION_CHANNEL->blocking_read(0.1);    //100ms
//        if(nml_read_ret == GLOBALPOSITIONINFO_MSG_TYPE) {
//            pose_queue->push(Pose(GlobalPositionInfo_data->Position.reserved[0], GlobalPositionInfo_data->Position.reserved[1],
//                    GlobalPositionInfo_data->Position.reserved[2], GlobalPositionInfo_data->Position.azimuth,
//                    GlobalPositionInfo_data->Position.pitch, GlobalPositionInfo_data->Position.roll),
//                    GlobalPositionInfo_data->local_PC_time);
//        }
//    }
//}

void PoseQueueInterface::push(const Pose& pose, double timestamp)
{
    pose_queue->push(pose, timestamp);
}

void PoseQueueInterface::poseAt(double time, Pose &pose)
{
    pthread_mutex_lock(&m_Mutex);
    pose_queue->pose(time, pose);
    pthread_mutex_unlock(&m_Mutex);
}

void PoseQueueInterface::latestPosition(PositionData &position)
{
    pthread_mutex_lock(&m_Mutex);
    position = GlobalPositionInfo_data->Position;
    pthread_mutex_unlock(&m_Mutex);
}
