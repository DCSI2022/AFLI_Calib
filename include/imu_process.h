/// IMU data process
/// Author: Weitong Wu, wwtgeomatics@gmail.com
/// Version 1.0
/// Create Time: 2022/5/30
/// Update Time:

#ifndef HELMET_CALIB_IMU_PROCESS_H
#define HELMET_CALIB_IMU_PROCESS_H

#include "types.h"

void find_imu_data_in_one_scan(const double &scan_start_time, const double &scan_end_time,
                               const std::vector<ImuData> &imu_data,
                               int &imu_data_start_index, int &imu_data_end_index,
                               std::vector<ImuData> &imu_data_in_one_scan);

// static initialization
void imu_initialization(const std::vector<ImuData> &imu_data, gtsam::Pose3 &T_I0toG,
                        Eigen::Vector3d &b_w0, Eigen::Vector3d &b_a0);

// using gyro measurement to integrate rotation for a short period
void rotation_integration (const std::vector<ImuData> &data, double imu_dt, gtsam::Rot3 &rotation);

#endif//HELMET_CALIB_IMU_PROCESS_H
