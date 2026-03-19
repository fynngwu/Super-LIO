//
// Offline wrapper for Super-LIO - replaces ROSWrapper for offline processing
//

#ifndef SUPER_LIO_OFFLINE_WRAPPER_H
#define SUPER_LIO_OFFLINE_WRAPPER_H

#include <deque>
#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "lio/super_lio.h"
#include "lio/params.h"
#include "lio/ESKF.h"

namespace LI2Sup {

/**
 * @brief Offline data wrapper that provides data buffering and synchronization
 *        similar to ROSWrapper but without ROS node dependency
 */
class OfflineWrapper : public DataWrapperBase {
public:
    using Ptr = std::shared_ptr<OfflineWrapper>;

    OfflineWrapper();
    ~OfflineWrapper();

    /// Set ESKF pointer
    void setESKF(ESKF::Ptr& eskf) override { eskf_ = eskf; }

    /// Synchronize IMU and LiDAR measurements
    bool sync_measure(MeasureGroup& meas) override;

    /// Publish/save odometry (called from SuperLIO::Output)
    void pub_odom(const NavState& state) override;

    /// Save trajectory to file
    void saveTrajectory(const std::string& filename);

    /// Clear buffers
    void clear();

    /// Push IMU data (called from bag reader callback)
    void pushImuData(const IMUDataPtr& imu);

    /// Push Livox LiDAR data (called from bag reader callback)
    void pushLivoxData(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg);

    /// Push standard PointCloud2 data (called from bag reader callback)
    void pushPointCloud2Data(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);

    /// Get last IMU timestamp
    double getLastImuTimestamp() const { return last_timestamp_imu_; }

    /// Get last LiDAR timestamp
    double getLastLidarTimestamp() const { return last_timestamp_lidar_; }

private:
    std::deque<IMUData> imu_buffer_;
    std::deque<LidarData> lidar_buffer_;
    
    bool lidar_pushed_ = false;
    double last_timestamp_imu_ = -1.0;
    double last_timestamp_lidar_ = -1.0;

    ESKF::Ptr eskf_{nullptr};
};

}  // namespace LI2Sup

#endif  // SUPER_LIO_OFFLINE_WRAPPER_H