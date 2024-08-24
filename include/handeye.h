/// hand eye calibration
/// Author: Weitong Wu, wwtgeomatics@gmail.com
/// Version 1.1
/// Create Time: 2021
/// Update Time: 2023/5/28

#ifndef SRC_HANDEYE_H
#define SRC_HANDEYE_H

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>

/*
 * @Input: 2 sensor trajectory (Quaternion and position) with time
 * @Output: extrinsic: r_2To1, p_2To1
 */
void hand_eye_calibration(vector<Eigen::Quaterniond> &sensor1_q_vec, vector<Eigen::Vector3d> &sensor1_p_vec, vector<double> &sensor1_time_vec,
                          vector<Eigen::Quaterniond> &sensor2_q_vec, vector<Eigen::Vector3d> &sensor2_p_vec, vector<double> &sensor2_time_vec,
                          Eigen::Matrix3d &r_2To1, Eigen::Vector3d &p_2To1);

/*
 * @Input: 2 sensor relative rotation seq (gtsam::Rot3)
 * @Output: rotation extrinsic: r_2To1
 */
void hand_eye_calibration_rotation(vector<gtsam::Rot3> &vec_relaRota1, vector<gtsam::Rot3> &vec_relaRota2, Eigen::Matrix3d &r_2To1);

#endif //SRC_HANDEYE_H
