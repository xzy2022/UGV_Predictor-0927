#ifndef POSE_QUEUE_H_
#define POSE_QUEUE_H_

#include <map>
#include <stdint.h>

#include "Pose.h"

class PoseQueue {
public:
    PoseQueue(uint32_t queue_size, bool extrapolate_data = false, double extrapolation_time_threshold = 0.0);
    virtual ~PoseQueue();

    bool empty() const {return queue_.empty();}
    void clear() {queue_.clear();}
    size_t size() const {return queue_.size();}
    size_t maxSize() const {return queue_size_;}

    double extrapolationTimeTreshold() {return extrapolation_time_threshold_;}
    void extrapolationTimeTreshold(double extrapolation_time_threshold) {extrapolation_time_threshold_ = extrapolation_time_threshold;}
    void push(const Pose& pose, double timestamp);



    void pose(double timestamp, Pose &res_pose);

    Pose latestPose();
    void latestPose(Pose& latest_pose, double& latest_timestamp);
    double latestPoseTimestamp();


private:
    int bracketPose(double timestamp,
                    std::map<double, Pose>::const_iterator& before, std::map<double, Pose>::const_iterator& after);
//    double interpolateYaw(double head1, double head2, double fraction);

    ///////////////////////////////////////////////////////////
    std::map <double, Pose> queue_;
    ///////////////////////////////////////////////////////////

    size_t queue_size_;
    bool extrapolate_data_;
    double extrapolation_time_threshold_;
};


#endif
