#ifndef KEYFRAME_CONTAINNER_HPP
#define KEYFRAME_CONTAINNER_HPP

#include "predefined_types.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <builtin_interfaces/msg/time.hpp>

#include <vector>

// Helper function to convert builtin_interfaces::msg::Time to nanoseconds
inline uint64_t timeToNSec(const builtin_interfaces::msg::Time& t) {
    return static_cast<uint64_t>(t.sec) * 1000000000ULL + static_cast<uint64_t>(t.nanosec);
}

class KeyFrame
{
public:
  KeyFrame() {
    pose_opt_set = false;
    this->KeyCloud.reset(new pcl::PointCloud<PointType>());
  }
  ~KeyFrame() {}

  Pose6D KeyPose;
  Pose6D KeyPoseOpt;
  Pose6D KeyPoseCompare;
  bool pose_opt_set;
  builtin_interfaces::msg::Time KeyTime;
  pcl::PointCloud<PointType>::Ptr KeyCloud;

private:

};


#endif // KEYFRAME_CONTAINNER_HPP
