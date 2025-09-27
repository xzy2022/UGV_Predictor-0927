
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>

#include "PoseQueue.h"

using std::cerr;

PoseQueue::PoseQueue(uint32_t queue_size, bool extrapolate_data, double extrapolation_time_threshold) :
    extrapolate_data_(extrapolate_data), extrapolation_time_threshold_(extrapolation_time_threshold)
{
    if(queue_size == 0)
        cerr<<"Invalid queue size (zero) requested.";
    queue_size_ = queue_size;
}

PoseQueue::~PoseQueue()
{

}

void PoseQueue::push(const Pose& pose, double timestamp)
{
    if(queue_.size() >= queue_size_)
        queue_.erase(queue_.begin());
    std::pair<std::map<double, Pose>::iterator, bool> rqp = queue_.insert(std::make_pair(timestamp, pose));
}

int PoseQueue::bracketPose(double timestamp, std::map<double, Pose>::const_iterator& before, std::map<double, Pose>::const_iterator& after)
{
    std::map<double, Pose>::const_iterator pit = queue_.lower_bound(timestamp);

    if (pit == queue_.end()) {
        if (queue_.empty())
            cerr<<"Pose queue is empty.";

        // requested timestamp is newer than newest entry in pose queue
        before = --queue_.end(); // TODO: implement proper extrapolation
        after = before;
        after--;
        return 1;
    }

    after = pit; // btw...who named lower_bound ?!?

    if (pit == queue_.begin()) {
        if (pit->first == timestamp) {
            before = pit;
            return -1;
        }

        // requested timestamp is older than oldest entry in pose queue
        before = queue_.begin();
        after = queue_.begin();
        after++;
        return -1;
    }

    // requested timestamp is somewhere between entries in pose queue
    before = --pit;
    return 0;
}


void PoseQueue::pose(double timestamp, Pose &res_pose)
{
    // empty case is handled in bracketPose
    if (queue_.size() <= 1) {
        res_pose = queue_.begin()->second;
        return;
    }

    std::map<double, Pose>::const_iterator before, after;
    int polation_mode = bracketPose(timestamp, before, after);
    Pose pose1 = before->second;
    Pose pose2 = after->second;

    double ts1 = before->first;
    double ts2 = after->first;
    if (polation_mode != 0) {
        if (!extrapolate_data_ || before->first == after->first) {
            // pose1 is always the one closest to requested ts
            res_pose = pose1;
            return;
        }
        if (std::abs(timestamp - ts1) > extrapolation_time_threshold_) {
            std::stringstream s;
            s << "Cannot determine pose for requested timestamp " << timestamp << "since it's too far off (dt=" << timestamp - ts1 << ")";
            cerr<<s.str();
            res_pose = pose1;
            return;
        }
        double dt = ts1 - ts2;

//        Pose res_pose;
        res_pose.x = pose1.x + (pose1.x - pose2.x) / dt * (timestamp - ts1);
        res_pose.y = pose1.y + (pose1.y - pose2.y) / dt * (timestamp - ts1);
        res_pose.z = pose1.z + (pose1.z - pose2.z) / dt * (timestamp - ts1);

        // TODO: implement proper angle extrapolation
        if( (pose1.azimuth > 30000) && (pose2.azimuth) < 10000)
            pose1.azimuth -= 36000;
        if( (pose1.azimuth < 10000) && (pose2.azimuth) > 30000)
            pose2.azimuth -= 36000;
        res_pose.azimuth = pose1.azimuth + (pose1.azimuth - pose2.azimuth) / dt * (timestamp - ts1);
        if(res_pose.azimuth >= 36000)
            res_pose.azimuth -= 36000;
        if(res_pose.azimuth < 0)
            res_pose.azimuth += 36000;

        res_pose.pitch = pose1.pitch + (pose1.pitch - pose2.pitch) / dt * (timestamp - ts1);
        res_pose.roll = pose1.roll + (pose1.roll - pose2.roll) / dt * (timestamp - ts1);
        return;
    }

    double frac = (timestamp - ts1) / (ts2 - ts1);

//    Pose res_pose;
    res_pose.x = pose1.x + frac * (pose2.x - pose1.x);
    res_pose.y = pose1.y + frac * (pose2.y - pose1.y);
    res_pose.z = pose1.z + frac * (pose2.z - pose1.z);

//    res_pose.azimuth = interpolateYaw(pose1.azimuth, pose2.azimuth, frac);
    if( (pose1.azimuth > 30000) && (pose2.azimuth) < 10000)
        pose1.azimuth -= 36000;
    if( (pose1.azimuth < 10000) && (pose2.azimuth) > 30000)
        pose2.azimuth -= 36000;
    res_pose.azimuth = pose1.azimuth + frac * (pose2.azimuth - pose1.azimuth);
    if(res_pose.azimuth >= 36000)
        res_pose.azimuth -= 36000;
    if(res_pose.azimuth < 0)
        res_pose.azimuth += 36000;

    res_pose.pitch = pose1.pitch + frac * (pose2.pitch - pose1.pitch);
    res_pose.roll = pose1.roll + frac * (pose2.roll - pose1.roll);

//    return res_pose;
}


//double PoseQueue::interpolateYaw(double head1, double head2, double fraction) {
//    double result;

//    if (head1 > 0 && head2 < 0 && head1 - head2 > M_PI) {
//        head2 += 2 * M_PI;
//        result = head1 + fraction * (head2 - head1);
//        if (result > M_PI) {
//            result -= 2 * M_PI;
//        }
//        return result;
//    }
//    else if (head1 < 0 && head2 > 0 && head2 - head1 > M_PI) {
//        head1 += 2 * M_PI;
//        result = head1 + fraction * (head2 - head1);
//        if (result > M_PI) {
//            result -= 2 * M_PI;
//        }
//        return result;
//    }

//    return head1 + fraction * (head2 - head1);
//}

void PoseQueue::latestPose(Pose& latest_pose, double& latest_timestamp) {
    if (queue_.empty())
        cerr<<"Pose queue is empty";

    std::map<double, Pose>::const_reverse_iterator latest = queue_.rbegin();
    latest_pose = latest->second;
    latest_timestamp = latest->first;
}

double PoseQueue::latestPoseTimestamp() {
    if (queue_.empty())
        cerr<<"Pose queue is empty";

    return queue_.rbegin()->first;
}

Pose PoseQueue::latestPose() {
    if (queue_.empty())
        cerr<<"Pose queue is empty";
    const Pose& tpose = queue_.rbegin()->second;
    return tpose;
}

