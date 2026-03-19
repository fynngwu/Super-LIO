//
// Offline wrapper implementation for Super-LIO
//

#include "offline_wrapper.h"
#include "bag_io.h"
#include "ros_utils.h"

#include <glog/logging.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>
#include <iomanip>
#include "basic/logs.h"

namespace LI2Sup {

// Trajectory storage
static std::vector<std::pair<double, BASIC::SE3>> trajectory_;

OfflineWrapper::OfflineWrapper() {
    imu_buffer_.clear();
    lidar_buffer_.clear();
    trajectory_.clear();
}

OfflineWrapper::~OfflineWrapper() {
    // Trajectory will be saved explicitly via saveTrajectory()
}

void OfflineWrapper::pub_odom(const NavState& state) {
    // Store trajectory
    BASIC::SE3 pose(state.R.R_, state.p);
    trajectory_.emplace_back(state.timestamp, pose);
}

void OfflineWrapper::saveTrajectory(const std::string& filename) {
    if (trajectory_.empty()) {
        LOG(WARNING) << "No trajectory to save";
        return;
    }

    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        LOG(ERROR) << "Failed to open file: " << filename;
        return;
    }

    // TUM format: timestamp tx ty tz qx qy qz qw
    ofs << std::fixed << std::setprecision(6);
    for (const auto& [ts, pose] : trajectory_) {
        auto q = pose.quaternion();
        ofs << ts << " "
            << pose.t_[0] << " " << pose.t_[1] << " " << pose.t_[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    }
    ofs.close();

    LOG(INFO) << GREEN << " ---> Trajectory saved to: " << filename 
              << " (" << trajectory_.size() << " poses)" << RESET;
}

void OfflineWrapper::clear() {
    imu_buffer_.clear();
    lidar_buffer_.clear();
    lidar_pushed_ = false;
    last_timestamp_imu_ = -1.0;
    last_timestamp_lidar_ = -1.0;
}

void OfflineWrapper::pushImuData(const IMUDataPtr& imu) {
    if (imu->secs < last_timestamp_imu_) {
        LOG(WARNING) << "IMU loop back, clear buffer";
        imu_buffer_.clear();
        last_timestamp_imu_ = imu->secs;
    }
    imu_buffer_.push_back(*imu);
    last_timestamp_imu_ = imu->secs;
}

void OfflineWrapper::pushLivoxData(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg) {
    if (msg->point_num < 10) return;
    
    LidarData lidar_data;
    std::size_t ptsize = msg->point_num;
    lidar_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());
    lidar_data.pc->reserve(ptsize / g_filter_rate + 1);

    double offset_time = 0.0;
    for (std::size_t i = 0; i < ptsize; i += g_filter_rate) {
        auto& pt = msg->points[i];
        auto tag = pt.tag & 0x30;
        if (tag == 0x10 || tag == 0x00) {
            auto dis = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
            if (dis > g_blind2 && dis < g_maxrange2) {
                offset_time = pt.offset_time * 1e-9;
                lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.reflectivity, offset_time);
            }
        }
    }
    lidar_data.start_time = ToSec(msg->header.stamp);
    lidar_data.end_time = lidar_data.start_time + offset_time;
    lidar_buffer_.push_back(lidar_data);
}

void OfflineWrapper::pushPointCloud2Data(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    if (msg->data.size() < 10) return;

    LidarData lidar_data;
    lidar_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());

    double offset_time = 0.0;

    // Helper function for valid point check
    auto validPoint = [](double x, double y, double z) -> bool {
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            return false;
        double d2 = x * x + y * y + z * z;
        return (d2 > g_blind2 && d2 < g_maxrange2);
    };

    switch (g_lidar_type) {
    case LID_TYPE::HESAI16: {
        pcl::PointCloud<hesai_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
        const double time_begin = pl_orig.points[0].timestamp;
        lidar_data.start_time = time_begin;
        for (std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate) {
            auto& pt = pl_orig.points[i];
            if (!validPoint(pt.x, pt.y, pt.z)) continue;
            offset_time = pt.timestamp - time_begin;
            lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.intensity, offset_time);
        }
        lidar_data.end_time = time_begin + offset_time;
        break;
    }
    case LID_TYPE::VEL_NCLT: {
        pcl::PointCloud<NCLT::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
        lidar_data.start_time = ToSec(msg->header.stamp);
        for (std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate) {
            auto& pt = pl_orig.points[i];
            if (!validPoint(pt.x, pt.y, pt.z)) continue;
            offset_time = pt.time * 1e-6;
            lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, 1.0, offset_time);
        }
        lidar_data.end_time = lidar_data.start_time + offset_time;
        break;
    }
    case LID_TYPE::VELO16:
    case LID_TYPE::VELO32: {
        pcl::PointCloud<velodyne_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
        lidar_data.start_time = ToSec(msg->header.stamp);
        for (std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate) {
            auto& pt = pl_orig.points[i];
            if (!validPoint(pt.x, pt.y, pt.z)) continue;
            lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.intensity, pt.time);
        }
        lidar_data.end_time = lidar_data.start_time + lidar_data.pc->points.back().offset_time;
        break;
    }
    case LID_TYPE::OUSTER: {
        pcl::PointCloud<ouster_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
        lidar_data.start_time = ToSec(msg->header.stamp);
        for (std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate) {
            auto& pt = pl_orig.points[i];
            if (!validPoint(pt.x, pt.y, pt.z)) continue;
            offset_time = pt.t * 1e-9;
            lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.intensity, offset_time);
        }
        lidar_data.end_time = lidar_data.start_time + offset_time;
        break;
    }
    default:
        return;
    }

    lidar_buffer_.push_back(lidar_data);
}

bool OfflineWrapper::sync_measure(MeasureGroup& meas) {
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }

    if (!lidar_pushed_) {
        meas.lidar = lidar_buffer_.front();
        lidar_pushed_ = true;
    }

    if (last_timestamp_lidar_ > meas.lidar.end_time) {
        lidar_buffer_.pop_front();
        lidar_pushed_ = false;
        return false;
    }

    if (last_timestamp_imu_ < meas.lidar.end_time) {
        return false;
    }

    double imu_time = imu_buffer_.front().secs;
    meas.imu.clear();
    while ((!imu_buffer_.empty()) && (imu_time < meas.lidar.end_time)) {
        imu_time = imu_buffer_.front().secs;
        if (imu_time > meas.lidar.end_time) break;
        meas.imu.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
    }

    last_timestamp_lidar_ = meas.lidar.end_time;
    lidar_buffer_.pop_front();
    lidar_pushed_ = false;
    return true;
}

}  // namespace LI2Sup