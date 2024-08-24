/// Data (rosbag, traj, etc.) IO
/// Author: Weitong Wu, wwtgeomatics@gmail.com
/// Version 1.1
/// Create Time: 2022/5/24
/// Update Time: 2023/5/8

#ifndef HELMET_CALIB_IO_H
#define HELMET_CALIB_IO_H

#include <livox_ros_driver/CustomMsg.h>
#include "types.h"

///@brief Parse livox custommsg into custom LidarData
///
/// \param[in] msg in livox_ros_driver::CustomMsgConstPtr format
/// \param[in] point_filter_num in int format
/// \param[out] one_frame_lidar_data in LidarData format
void parseLivoxLidar(const livox_ros_driver::CustomMsgConstPtr &msg, int point_filter_num, LidarData &one_frame_lidar_data);

///@brief load vector of lidardata from rosbag
///
/// \param[in] in_bag_file in std::string format
/// \param[in] lidar_type in int format
/// \param[in] point_filter_num in int format
/// \param[in] topics in std::vector<std::string>
/// \param[out] lidar_data in std::vector LidarData format
void loadLidarDataFromRosbag(std::string rosbag_path, int lidar_type, int point_filter_num,
                             std::vector<std::string> &topics,
                             float start_time, float dur_time,
                             std::vector<LidarData> &lidar_data);

void loadLidarDataFromFolder(std::string fold_path, int point_filter_num, std::vector<LidarData> &scan_vec);

void loadImuDataFromRosbag(std::string rosbag_path, std::vector<ImuData> &imu_data, double &dt);

void writePoseFile(std::string path, std::vector<PoseData> &pose_data);

void loadPoseFile(std::string pose_path, std::vector<PoseData> &pose_data);

#endif//HELMET_CALIB_IO_H
