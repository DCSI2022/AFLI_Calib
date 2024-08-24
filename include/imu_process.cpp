/// IMU data process
/// Author: Weitong Wu, wwtgeomatics@gmail.com
/// Version 1.0
/// Create Time: 2022/5/30
/// Update Time:

#include "imu_process.h"

void find_imu_data_in_one_scan(const double &scan_start_time, const double &scan_end_time,
                               const std::vector<ImuData> &imu_data,
                               int &imu_data_start_index, int &imu_data_end_index,
                               std::vector<ImuData> &imu_data_in_one_scan) {
    ///find imu data index that just smalled than scan_start_time and just larger than scan_end_time
    for (int j = imu_data_start_index; j < imu_data.size(); ++j) {
        imu_data_start_index = j;
        if (imu_data[j].timestamp_s > scan_start_time) {
            imu_data_start_index--;
            break;
        }
    }
    for (int j = imu_data_start_index + 1; j < imu_data.size(); ++j) {
        if (imu_data[j].timestamp_s > scan_end_time) {
            imu_data_end_index = j - 1;
            break;
        }
    }
    if (imu_data_start_index == 0 || imu_data_end_index == 0) {
        std::cout << "couldn't fine imu index" << std::endl;
        return;
    }
    ///Get imu data in one scan
    for (int j = imu_data_start_index; j < imu_data_end_index + 1; ++j) {
        imu_data_in_one_scan.push_back(imu_data[j]);
    }
}

void imu_initialization(const std::vector<ImuData> &imu_data, gtsam::Pose3 &T_I0toG,
                        Eigen::Vector3d &b_w0, Eigen::Vector3d &b_a0)
{
    Eigen::Vector3d a_avg1(0, 0, 0);
    Eigen::Vector3d w_avg1(0, 0, 0);
    for (const ImuData &data: imu_data) {
        a_avg1 += data.am;
        w_avg1 += data.wm;
    }
    a_avg1 /= imu_data.size();
    cout << "a_avg: " << a_avg1 << std::endl;
    w_avg1 /= imu_data.size();

    // Get z axis, which is g in I0
    Eigen::Vector3d z_axis = a_avg1 / a_avg1.norm();

    // Create an x_axis
    Eigen::Vector3d e_1(1, 0, 0);

    // Make x_axis perpendicular to z (Schmidt orthogonalization)
    /// Note: In the orientation initialization, a_avg1 is only information we can get and use, so that z_axis can be determined by it, but x_axis is dertermined by user.
    /// right here, we use Schmidt orthogonalization among z_axis and (1,0,0) to get the x_axis
    Eigen::Vector3d x_axis = e_1 - z_axis * z_axis.transpose() * e_1;
    x_axis = x_axis / x_axis.norm();

    ///Get y from the cross product of these two
    Eigen::Vector3d y_axis = z_axis.cross(x_axis);

    ///From these axes get rotation
    Eigen::Matrix3d r_GtoI0;
    r_GtoI0.block(0, 0, 3, 1) = x_axis;
    r_GtoI0.block(0, 1, 3, 1) = y_axis;
    r_GtoI0.block(0, 2, 3, 1) = z_axis;
    T_I0toG = gtsam::Pose3(gtsam::Rot3(r_GtoI0.inverse()), gtsam::Point3());
    cout << "T_I0toG: " << std::endl;
    T_I0toG.print();

    ///Set our biases equal to our noise
    b_w0 = w_avg1;
    b_a0 = Eigen::Vector3d(0, 0, 0);
}

void rotation_integration (const std::vector<ImuData> &data, double imu_dt, gtsam::Rot3 &rotation)
{
    auto w0 = data[0].wm;
    for (int i = 1; i < data.size(); i++) {
        auto w1 = data[i].wm;
        auto w_avg = 0.5 * (w0 + w1);
        auto rotation_vector = w_avg * imu_dt;
        rotation = rotation.expmap(rotation_vector);
        w0 = w1;
    }
}