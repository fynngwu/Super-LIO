//
// Adapted from lightweight_slam for Super-LIO offline processing
//

#ifndef SUPER_LIO_ROS_UTILS_H
#define SUPER_LIO_ROS_UTILS_H

#include <rclcpp/rclcpp.hpp>

namespace LI2Sup {

/// 时间转换工具
inline double ToSec(const builtin_interfaces::msg::Time &time) { 
    return static_cast<double>(time.sec) + 1e-9 * static_cast<double>(time.nanosec); 
}

inline uint64_t ToNanoSec(const builtin_interfaces::msg::Time &time) { 
    return static_cast<uint64_t>(time.sec) * 1000000000ULL + time.nanosec; 
}

}  // namespace LI2Sup

#endif  // SUPER_LIO_ROS_UTILS_H