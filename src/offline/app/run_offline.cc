//
// Super-LIO offline processing main program
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "lio/super_lio.h"
#include "lio/params.h"
#include "bag_io.h"
#include "offline_wrapper.h"
#include "yaml_io.h"

DEFINE_string(input_bag, "", "Input ROS2 bag path");
DEFINE_string(config, "", "Config YAML file path");

using namespace LI2Sup;

/// Load parameters from YAML file
void LoadParamsFromYaml(const std::string& config_path) {
    YAML_IO yaml(config_path);

    // Map parameters
    g_save_map = yaml.GetValue<bool>("lio", "map", "save_map");
    g_if_filter = yaml.GetValue<bool>("lio", "map", "if_filter");
    g_save_map_dir = yaml.GetValue<std::string>("lio", "map", "save_map_dir");
    g_map_name = yaml.GetValue<std::string>("lio", "map", "map_name");
    g_map_ds_size = yaml.GetValue<float>("lio", "map", "ds_size");
    g_pcd_save_interval = yaml.GetValue<int>("lio", "map", "save_interval");

    // ROS topics
    g_lidar_topic = yaml.GetValue<std::string>("lio", "ros", "lidar_topic");
    g_imu_topic = yaml.GetValue<std::string>("lio", "ros", "imu_topic");

    // Sensor parameters
    g_lidar_type = yaml.GetValue<int>("lio", "sensor", "lidar_type");
    double blind = yaml.GetValue<double>("lio", "sensor", "blind");
    g_blind2 = blind * blind;
    double maxrange = yaml.GetValue<double>("lio", "sensor", "maxrange");
    g_maxrange2 = maxrange * maxrange;
    g_filter_rate = yaml.GetValue<int>("lio", "sensor", "filter_rate");
    g_enable_downsample = yaml.GetValue<bool>("lio", "sensor", "enable_downsample");
    g_voxel_fliter_size = yaml.GetValue<float>("lio", "sensor", "voxel_fliter_size");
    g_gravity_norm = yaml.GetValue<double>("lio", "sensor", "gravity_norm");
    g_imu_type = yaml.GetValue<int>("lio", "sensor", "imu_type");
    g_imu_na = yaml.GetValue<double>("lio", "sensor", "imu_na");
    g_imu_ng = yaml.GetValue<double>("lio", "sensor", "imu_ng");
    g_imu_nba = yaml.GetValue<double>("lio", "sensor", "imu_nba");
    g_imu_nbg = yaml.GetValue<double>("lio", "sensor", "imu_nbg");

    // Extrinsic lidar-imu
    std::vector<double> extrinsic_lidar_imu = yaml.GetValue<std::vector<double>>("lio", "extrinsic", "lidar_imu");
    BASIC::V3 t_li(extrinsic_lidar_imu[0], extrinsic_lidar_imu[1], extrinsic_lidar_imu[2]);
    std::vector<BASIC::scalar> r_data(9);
    for (int i = 0; i < 9; ++i) {
        r_data[i] = static_cast<BASIC::scalar>(extrinsic_lidar_imu[3 + i]);
    }
    BASIC::M3 R_li(r_data.data());
    g_lidar_imu = BASIC::SE3(R_li, t_li);

    // Extrinsic odom-robo
    std::vector<double> extrinsic_odom_robo = yaml.GetValue<std::vector<double>>("lio", "extrinsic", "odom_robo");
    BASIC::V3 t_or(extrinsic_odom_robo[0], extrinsic_odom_robo[1], extrinsic_odom_robo[2]);
    auto temp_R = Eigen::AngleAxisd(extrinsic_odom_robo[5] * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(extrinsic_odom_robo[4] * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(extrinsic_odom_robo[3] * M_PI / 180.0, Eigen::Vector3d::UnitX());
    g_odom_robo.R_ = temp_R.toRotationMatrix().cast<BASIC::scalar>().transpose();
    g_odom_robo = BASIC::SE3(g_odom_robo.R_, t_or);

    auto temp_R_yaw = Eigen::AngleAxisd(extrinsic_odom_robo[5] * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    g_lidar_robo_yaw = temp_R_yaw.cast<BASIC::scalar>();

    // Hash map
    g_ivox_capacity = yaml.GetValue<int>("lio", "hash_map", "hash_capacity");
    g_ivox_resolution = yaml.GetValue<float>("lio", "hash_map", "vox_resolution");

    // KF
    g_kf_type = yaml.GetValue<int>("lio", "kf", "kf_type");
    g_kf_max_iterations = yaml.GetValue<int>("lio", "kf", "kf_max_iterations");
    g_kf_align_gravity = yaml.GetValue<bool>("lio", "kf", "kf_align_gravity");
    g_kf_quit_eps = yaml.GetValue<double>("lio", "kf", "kf_quit_eps");

    // Output
    g_2_robot = yaml.GetValue<bool>("lio", "output", "robot");
    g_2_plan_env_world = yaml.GetValue<bool>("lio", "output", "plan_env_world");
    g_2_plan_env_body = yaml.GetValue<bool>("lio", "output", "plan_env_body");
    g_2_ml_map = yaml.GetValue<bool>("lio", "output", "ml_map");
    g_visual_map = yaml.GetValue<bool>("lio", "output", "map");
    g_visual_dense = yaml.GetValue<bool>("lio", "output", "dense");
    g_pub_step = yaml.GetValue<int>("lio", "output", "pub_step");

    // Evaluation
    g_time_eva = yaml.GetValue<bool>("lio", "eva", "timer");

    LOG(INFO) << "Parameters loaded from: " << config_path;
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_input_bag.empty()) {
        LOG(ERROR) << "Usage: run_offline --input_bag=<bag_path> --config=<config.yaml>";
        return -1;
    }

    if (FLAGS_config.empty()) {
        LOG(ERROR) << "Please specify config file with --config=<path>";
        return -1;
    }

    // Load parameters
    LoadParamsFromYaml(FLAGS_config);

    // Create offline wrapper
    auto wrapper = std::make_shared<OfflineWrapper>();

    // Create SuperLIO instance
    SuperLIO slam;
    slam.setDataWrapper(wrapper);
    slam.init();

    LOG(INFO) << "Starting offline processing...";
    LOG(INFO) << "  Bag: " << FLAGS_input_bag;
    LOG(INFO) << "  Config: " << FLAGS_config;
    LOG(INFO) << "  Lidar topic: " << g_lidar_topic;
    LOG(INFO) << "  IMU topic: " << g_imu_topic;

    // Create bag reader
    RosbagIO rosbag(FLAGS_input_bag);

    // Add handlers - push data to wrapper buffer, then process
    rosbag.AddImuHandle(g_imu_topic, [&wrapper, &slam](IMUDataPtr imu) {
        wrapper->pushImuData(imu);
        // Try to process after each IMU data
        slam.process();
        return true;
    });

    if (g_lidar_type == LID_TYPE::LIVOX) {
        rosbag.AddLivoxCloudHandle(g_lidar_topic, [&wrapper, &slam](livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
            wrapper->pushLivoxData(msg);
            // Try to process after each LiDAR data
            slam.process();
            return true;
        });
    } else {
        rosbag.AddPointCloud2Handle(g_lidar_topic, [&wrapper, &slam](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            wrapper->pushPointCloud2Data(msg);
            // Try to process after each LiDAR data
            slam.process();
            return true;
        });
    }

    // Process bag
    rosbag.Go();

    // Save map if needed
    slam.saveMap();

    // Save trajectory
    wrapper->saveTrajectory("trajectory.txt");

    // Print timing info
    slam.printTimeRecord();

    LOG(INFO) << "Offline processing completed.";

    return 0;
}