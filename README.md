<div align="center">
  <h1>⚡Super-LIO</h1>
  <h2>Super-LIO: A Robust and Efficient LiDAR-Inertial Odometry System with a Compact Mapping Strategy</h2>
  <p><strong>This work has been accepted to <i> IEEE Robotics and Automation Letters (RA-L 2026)</i>.</strong></p>
  <br>

  [![Code](https://img.shields.io/badge/GitHub-181717?style=flat-square&logo=github&logoWidth=16)](https://github.com/Liansheng-Wang/Super-LIO.git) [![arXiv](https://img.shields.io/badge/arXiv-blue?logo=arxiv&color=%23B31B1B)](https://arxiv.org/abs/2509.05723) [![IEEE](https://img.shields.io/badge/RAL2026-004088.svg)](https://ieeexplore.ieee.org/document/11347459) [![Bilibili](https://img.shields.io/badge/Bilibili-00A1D6?style=flat-square&logo=bilibili&logoColor=white&logoWidth=16)](https://www.bilibili.com/video/BV11wBeBYEp6) [![YouTube](https://img.shields.io/badge/YouTube-FF0000?style=flat-square&logo=youtube&logoColor=white&logoWidth=16)](https://youtu.be/m9-hl8s5DDw)
</div>


<div align="center">
  <p>
    <a href="https://github.com/Liansheng-Wang/Super-LIO/tree/ros1" style="text-decoration: none;">
      <img src="https://img.shields.io/badge/🔄 SWITCH - ROS1 Noetic-3b82f6?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22" alt="Switch to ROS1"
      onmouseover="this.src='https://img.shields.io/badge/🔄 SWITCH - ROS1 Noetic-1e40af?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22'"
      onmouseout="this.src='https://img.shields.io/badge/🔄 SWITCH - ROS1 Noetic-3b82f6?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22'"/>
    </a>&nbsp;&nbsp;
    <a href="https://github.com/Liansheng-Wang/Super-LIO/tree/ros2" style="text-decoration: none;">
      <img src="https://img.shields.io/badge/✅ ACTIVE - ROS2 Humble/Iron/Jazzy-22c55e?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22" alt="ROS2 Active"
      onmouseover="this.src='https://img.shields.io/badge/✅ ACTIVE - ROS2 Humble/Iron/Jazzy-166534?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22'"
      onmouseout="this.src='https://img.shields.io/badge/✅ ACTIVE - ROS2 Humble/Iron/Jazzy-22c55e?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22'"/>
    </a>&nbsp;&nbsp;
    <a href="#" style="text-decoration: none; cursor: default;">
      <img src="https://img.shields.io/badge/🖥️ PLATFORM - X86 + ARM64-8b5cf6?style=for-the-badge&logo=linux&logoColor=white&logoWidth=22" alt="X86 and ARM Support"
      onmouseover="this.src='https://img.shields.io/badge/🖥️ PLATFORM - X86_64 + ARM64-4f46e5?style=for-the-badge&logo=linux&logoColor=white&logoWidth=22'"
      onmouseout="this.src='https://img.shields.io/badge/🖥️ PLATFORM - X86_64 + ARM64-8b5cf6?style=for-the-badge&logo=linux&logoColor=white&logoWidth=22'"/>
    </a>
  </p>
</div>

## Overview

<p align="center">
  <img src="docs/system_overview.png" width="95%">
</p>

**Key Features:  Efficient · Robust · Cross-Platform Compatible · Supports Both ROS1/ROS2 Versions**

Super-LIO is a robust and efficient LiDAR–Inertial Odometry (LIO) system designed for real-time and large-scale autonomous navigation. It introduces a compact and structured mapping strategy that enables predictable correspondence search and stable state estimation. The system is validated through extensive real-world experiments and comparisons with state-of-the-art methods, which demonstrates that Super-LIO not only achieves **excellent accuracy** but also maintains **lower resource consumption** and realizes a nearly **1.2–4× higher real-time processing speed**⚡.


**Contributors**: [Liansheng Wang](https://github.com/Liansheng-Wang), [Xinke Zhang](https://github.com/PSQzzzxk), [Chenhui Li](https://github.com/kermitLHH), [Dongjiao He](https://github.com/Joanna-HE), [Yihan pan](https://github.com/pyh3552), Jianjun Yi.


## Quickly Run

**For ROS1 Users**: Please switch to the **ros1** branch and follow the instructions at [ros1 branch](https://github.com/Liansheng-Wang/Super-LIO/tree/ros1)

### Requirements

Ubuntu 24(22).04 · C++20 · ROS Jazzy(Humble) · Eigen · PCL 

### Dependencies

glog · TBB

```bash
sudo apt install libgoogle-glog-dev libtbb-dev
```

### Build & Run
```bash
git clone https://github.com/Liansheng-Wang/Super-LIO.git
cd Super-LIO
colcon build

source install/setup.bash
ros2 launch super_lio Livox_mid360.py

```

#### 🔁 Relocalization Mode
Super-LIO supports relocalization using a pre-built map, allowing the system to resume localization from a saved map without restarting the mapping process.
This mode is useful for long-term deployment, repeated missions, or recovery after tracking loss.

Before running relocalization, please make sure that:
- A map has been previously saved to disk.

```bash
cd PATH_2_Super-LIO
source install/setup.bash
ros2 launch super_lio relocation.py
```


## Offline Processing

Super-LIO supports offline processing mode, allowing you to run SLAM directly on ROS2 bag files without launching ROS nodes. This is useful for algorithm testing and evaluation.

### Features

- Direct ROS2 bag loading without ROS node dependency
- Outputs trajectory in TUM format
- Supports all LiDAR types (Livox, Velodyne, Ouster, Hesai, etc.)
- Independent CMake build system

### Build

```bash
cd Super-LIO/src/offline
./build.sh
```

### Usage

```bash
cd Super-LIO/src/offline
./build/run_offline --input_bag=<bag_path> --config=config/offline.yaml
```

### Output

- **Trajectory**: Saved to `trajectory.txt` in TUM format (timestamp tx ty tz qx qy qz qw)
- **Map**: Saved to the directory specified in config (if `save_map` is enabled)

### Configuration

The offline config file is located at `src/offline/config/offline.yaml`. Key parameters:

```yaml
lio:
  ros:
    lidar_topic: "/livox/lidar"
    imu_topic: "/livox/imu"
  
  sensor:
    lidar_type: 1  # 1=Livox, 2=Velodyne16, 3=Velodyne32, 4=Ouster, 5=Hesai, 6=NCLT
    blind: 2.0
    maxrange: 60.0
  
  map:
    save_map: false
    save_map_dir: "map"
```

### Example

```bash
# Run with Livox bag
./build/run_offline --input_bag=/path/to/livox_bag --config=config/offline.yaml

# Output:
# ---> Trajectory saved to: trajectory.txt (1167 poses)
```

### Point-wise Update Mode

Super-LIO supports two Kalman Filter update modes:
- **Batch update (default)**: `point_update: false` - All points in a scan are processed together
- **Sequential point-wise update**: `point_update: true` - Each point is used to update the state sequentially

The point-wise mode can provide more accurate results in certain scenarios but is computationally more expensive.

```yaml
lio:
  kf:
    kf_type: 0
    kf_max_iterations: 4
    kf_align_gravity: true
    kf_quit_eps: 0.001
    point_update: false  # true=sequential point-wise, false=batch
```

### Testing Scripts

Super-LIO provides testing scripts for running experiments with different parameter combinations.

#### Quick Test (run_test.sh)

Runs 2 quick experiments comparing `point_update=true` vs `point_update=false`:

```bash
cd Super-LIO/src/offline
./scripts/run_test.sh
```

**Output:**
```
========================================
Super-LIO 快速测试脚本
========================================
数据集: /home/wufy/git_resp/ros2_ws/src/court1-ros2bag
...
实验 1: point_update=true
输出目录: experiment_results/exp_1_putrue_vs0.5_ki4
...
实验 2: point_update=false
输出目录: experiment_results/exp_2_pufalse_vs0.5_ki4
...
测试完成！
```

Each experiment generates:
- `config.yaml` - Configuration used
- `params.txt` - Parameter summary
- `run.log` - Execution log
- `trajectory.txt` - Output trajectory (TUM format)

#### Orthogonal Experiments (run_experiments.sh)

Runs a full grid of parameter combinations for systematic evaluation:

```bash
cd Super-LIO/src/offline
# Full factorial (18 combinations)
./scripts/run_experiments.sh --full

# Quick mode (2 combinations)
./scripts/run_experiments.sh --quick

# Custom bag path
./scripts/run_experiments.sh --bag=/path/to/bag --full
```

**Parameters tested:**
- `point_update`: true, false
- `voxel_fliter_size`: 0.3, 0.5, 0.7 (--full mode)
- `kf_max_iterations`: 2, 4, 6 (--full mode)

**Output structure:**
```
experiment_results/
├── exp_1_putrue_vs0.5_ki4/
│   ├── config.yaml
│   ├── params.txt
│   ├── run.log
│   ├── trajectory.txt
│   └── trajectory_plot.png
├── exp_2_pufalse_vs0.5_ki4/
│   └── ...
├── summary.txt          # Summary of all experiments
└── comparison_plot.png   # Compare results
```

#### Analysis Script (analyze_results.py)

Analyzes experimental results and generates visualizations:

```bash
# Analyze single experiment
python3 scripts/analyze_results.py experiment_results/exp_1_putrue_vs0.5_ki4

# Compare all experiments in a directory
python3 scripts/analyze_results.py experiment_results
```

**Output includes:**
- Trajectory statistics (total distance, duration, avg/max speed)
- Time performance breakdown (Observe, DownSample, Undistort, UpdateMap)
- 3D trajectory plots
- Speed distribution plots
- Comparison bar charts between experiments

```
============================================================
实验对比表格
============================================================

实验名称                          Observe(ms)  point_update  距离(m)     时间(s)
--------------------------------------------------------------------------------
exp_1_putrue_vs0.5_ki4            12.345       true           523.45     120.67
exp_2_pufalse_vs0.5_ki4          8.234        false          521.89     118.45
```

## Datasets
<p align="center">
  <img src="docs/datasets_compressed.png" width="95%">
</p>

Super-LIO is evaluated on multiple real-world datasets covering diverse environments,
including indoor, outdoor, and large-scale scenes.

> **TODO**: Dataset download links and detailed descriptions will be provided in the future.


---

## Publications

If your like our projects, please cite us and support us with a star 🌟.
We kindly recommend to cite [our paper](https://ieeexplore.ieee.org/document/11347459) if you find this library useful:

```latex
@article{wang2026superlio,
  title   = {Super-LIO: A Robust and Efficient LiDAR-Inertial Odometry System with a Compact Mapping Strategy},
  author  = {Wang, Liansheng and Zhang, Xinke and Li, Chenhui and He, Dongjiao and Pan, Yihan and Yi, Jianjun},
  journal = {IEEE Robotics and Automation Letters},
  year    = {2026},
  volume  = {11},
  number  = {3},
  pages   = {2666--2673},
  doi     = {10.1109/LRA.2026.3653372}
}
```


## Update Logs

<details>
<summary>Click to expand <b>Update Logs</b> (click to collapse)</summary>

<br>

- 2026-01-04  
  - Separate ROS interface and algorithm.
  - Refactor SuperLIOReLoc to inherit from SuperLIO.
  - Code style aligned with ROS2 version.

- 2026-01-04
  - The main branch is renamed to ros1
  - add ros2 branch

- 2026-01-04 21:51
  - release ROS2 version

</details>