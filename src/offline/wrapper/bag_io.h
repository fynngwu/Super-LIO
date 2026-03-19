//
// Adapted from lightweight_slam for Super-LIO offline processing
//

#ifndef SUPER_LIO_BAG_IO_H
#define SUPER_LIO_BAG_IO_H

#include <functional>
#include <map>
#include <string>
#include <memory>
#include <csignal>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "livox_ros_driver2/msg/custom_msg.hpp"

#include "basic/alias.h"
#include "common/ds.h"

namespace LI2Sup {

// 全局退出标志
inline std::atomic<bool>& GetExitFlag() {
    static std::atomic<bool> flag{false};
    return flag;
}

inline void SigHandle(int sig) {
    GetExitFlag() = true;
}

/**
 * ROSBAG IO
 * 指定一个包名，添加一些回调函数，就可以顺序遍历这个包
 */
class RosbagIO {
   public:
    explicit RosbagIO(std::string bag_file)
        : bag_file_(std::move(bag_file)) {
        signal(SIGINT, SigHandle);
    }

    using MsgType = std::shared_ptr<rosbag2_storage::SerializedBagMessage>;
    using MessageProcessFunction = std::function<bool(const MsgType &m)>;

    /// 回调类型定义
    using PointCloud2Handle = std::function<bool(sensor_msgs::msg::PointCloud2::SharedPtr)>;
    using LivoxCloud2Handle = std::function<bool(livox_ros_driver2::msg::CustomMsg::SharedPtr)>;
    using ImuHandle = std::function<bool(IMUDataPtr)>;

    /// 遍历文件内容，调用回调函数
    void Go();

    /// 通用处理函数
    RosbagIO &AddHandle(const std::string &topic_name, MessageProcessFunction func) {
        process_func_.emplace(topic_name, func);
        return *this;
    }

    /// point cloud 2 处理
    RosbagIO &AddPointCloud2Handle(const std::string &topic_name, PointCloud2Handle f) {
        return AddHandle(topic_name, [f, this](const MsgType &m) -> bool {
            auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            rclcpp::SerializedMessage data(*m->serialized_data);
            seri_cloud2_.deserialize_message(&data, msg.get());
            return f(msg);
        });
    }

    /// livox 处理
    RosbagIO &AddLivoxCloudHandle(const std::string &topic_name, LivoxCloud2Handle f) {
        return AddHandle(topic_name, [f, this](const MsgType &m) -> bool {
            auto msg = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
            rclcpp::SerializedMessage data(*m->serialized_data);
            seri_livox_.deserialize_message(&data, msg.get());
            return f(msg);
        });
    }

    /// IMU 处理 - 适配 Super-LIO 的 IMUData
    RosbagIO &AddImuHandle(const std::string &topic_name, ImuHandle f) {
        return AddHandle(topic_name, [f, this](const MsgType &m) -> bool {
            auto msg = std::make_shared<sensor_msgs::msg::Imu>();
            rclcpp::SerializedMessage data(*m->serialized_data);
            seri_imu_.deserialize_message(&data, msg.get());

            IMUDataPtr imu = std::make_shared<IMUData>();
            imu->secs = static_cast<double>(msg->header.stamp.sec) + 
                        static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
            imu->acc = BASIC::V3(msg->linear_acceleration.x,
                                 msg->linear_acceleration.y,
                                 msg->linear_acceleration.z);
            imu->gyr = BASIC::V3(msg->angular_velocity.x,
                                  msg->angular_velocity.y,
                                  msg->angular_velocity.z);
            return f(imu);
        });
    }

    /// 清除现有的处理函数
    void CleanProcessFunc() { process_func_.clear(); }

   private:
    std::map<std::string, MessageProcessFunction> process_func_;

    /// 序列化器
    rclcpp::Serialization<sensor_msgs::msg::Imu> seri_imu_;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> seri_cloud2_;
    rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> seri_livox_;

    std::string bag_file_;
};

}  // namespace LI2Sup

#endif  // SUPER_LIO_BAG_IO_H