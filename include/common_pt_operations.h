/// Common point operations and other functions
/// Author: Weitong Wu, wwtgeomatics@gmail.com
/// Version 1.1
/// Create Time: 2022/5/23
/// Update Time: 2023/5/28

#ifndef HELMET_CALIB_COMMON_PT_OPERATIONS_H
#define HELMET_CALIB_COMMON_PT_OPERATIONS_H

#include <gtsam/geometry/Pose3.h>
#include <pcl/filters/voxel_grid.h>
#include "types.h"
#include <boost/filesystem.hpp>
#include <unordered_map>

/// @brief Calculate distance between two 3d points
///
/// @param[in] pt1 in PointXYZ* format
/// @param[in] pt2 in PointXYZ* format
/// @return distance in float format
template<typename T>
inline float point2pointDistance(T pt1, T pt2) {
    float dx = pt1.x - pt2.x;
    float dy = pt1.y - pt2.y;
    float dz = pt1.z - pt2.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

/// @brief Transform point with given pose
///
/// @param[in] pt_in in pcl::Point* format
/// @param[in] pose in gtsam::Pose3 format
/// @param[out] pt_out in pcl::Point* format
template<typename T>
inline void transformPoint(T &pt_in, gtsam::Pose3 pose, T &pt_out) {
    pt_out = pt_in;///first copy
    gtsam::Point3 pt(pt_in.x, pt_in.y, pt_in.z);
    auto pt_world = pose.transformFrom(pt);
    pt_out.x = pt_world.x();
    pt_out.y = pt_world.y();
    pt_out.z = pt_world.z();
}

/// @brief Transform pointcloud with given pose
///
/// @param[in] pointcloud_in in pcl::PointCloud<*>::Ptr format
/// @param[in] pose in Pose3 format
/// @param[out] pointcloud_out in pcl::PointCloud<*>::Ptr format
template<typename T>
void transformPointcloud(T pointcloud_in, gtsam::Pose3 pose, T pointcloud_out) {
    pointcloud_out->resize(pointcloud_in->points.size());
    for (int j = 0; j < pointcloud_in->points.size(); ++j) {
        transformPoint(pointcloud_in->points[j], pose, pointcloud_out->points[j]);
    }
}

/// @brief Voxel grid filter for PointCloud
///
/// @param[in/out] pointcloudptr in pcl::PointCloud<T>::Ptr format
/// @param[in] grid_size
/// @tips need specify T when call this function
template<typename T>
void voxelGridFilter(typename pcl::PointCloud<T>::Ptr pointcloudptr, float grid_size) {
    pcl::VoxelGrid<T> filter;
    filter.setLeafSize(grid_size, grid_size, grid_size);
    filter.setInputCloud(pointcloudptr);
    filter.filter(*pointcloudptr);
}

/// @brief Voxel grid filter for PointCloud, reserve the point nearest to the voxel center
///
/// @param[in] pointcloudptr in pcl::PointCloud<T>::Ptr format
/// @param[in] grid_size
/// @param[out] pointcloudptr_downsample in pcl::PointCloud<T>::Ptr format
/// @tips need specify T when call this function
template<typename T>
void voxelGridFilter1(typename pcl::PointCloud<T>::Ptr pointcloudptr, float grid_size,
                      typename pcl::PointCloud<T>::Ptr pointcloudptr_downsample) {
    // 创建一个 hash 表，用于快速查找每个点所在的 voxel 索引
    std::unordered_map<size_t, std::vector<size_t>> hash;
    for (size_t i = 0; i < pointcloudptr->points.size(); ++i) {
        T pt = pointcloudptr->points[i];
        size_t idx = static_cast<size_t>(std::floor(pt.x / grid_size)) +
                     (static_cast<size_t>(std::floor(pt.y / grid_size)) << 10) +
                     (static_cast<size_t>(std::floor(pt.z / grid_size)) << 20);
        hash[idx].push_back(i);
    }

    // 遍历每个 voxel 中的点，保留每个 voxel 中距离中心最近的点
    for (const auto &kv: hash) {
        // 计算当前 voxel 的中心坐标
        float cx = (static_cast<size_t>(std::floor(kv.first)) & 0x3FF) * grid_size + grid_size / 2.0f;
        float cy = ((static_cast<size_t>(std::floor(kv.first)) >> 10) & 0x3FF) * grid_size + grid_size / 2.0f;
        float cz = ((static_cast<size_t>(std::floor(kv.first)) >> 20) & 0x3FF) * grid_size + grid_size / 2.0f;

        // 找出当前 voxel 中距离中心最近的点的索引
        float min_dist = std::numeric_limits<float>::max();
        size_t min_index = 0;
        for (size_t i = 0; i < kv.second.size(); ++i) {
            T pt = pointcloudptr->points[kv.second[i]];
            float dist = std::sqrt(std::pow(pt.x - cx, 2.0f) +
                                   std::pow(pt.y - cy, 2.0f) +
                                   std::pow(pt.z - cz, 2.0f));
            if (dist < min_dist) {
                min_dist = dist;
                min_index = kv.second[i];
            }
        }

        // 将距离最近的点加入输出点云中
        pointcloudptr_downsample->points.push_back(pointcloudptr->points[min_index]);
    }
}

///@brief calculate week sec given utc time in secs
///
/// \param[in] utc_sec in double format
/// \return week_sec in double format
inline double calUTCWeekSec(double utc_sec) {
    int week = (utc_sec + 4 * 24 * 3600) / (7 * 24 * 3600);
    double week_sec = utc_sec + 4 * 24 * 3600 - week * 7 * 24 * 3600;
    return week_sec;
}

/*
 * @input: rotation_matrix in Eigen::Matrix3d
 * @output: euler_angles in Eigen::Vector3d with rad
 * @notes: rotation_matrix = Rx(euler_angles[0]) * Ry(euler_angles[1]) * Rz(euler_angles[2])
 */
inline void rotationMatrix_to_eulerAngles(const Eigen::Matrix3d &rotation_matrix, Eigen::Vector3d &euler_angles) {
    euler_angles[1] = asin(rotation_matrix(0, 2));
    ///another result
    //euler_angles[1] = M_PI - asin(rotation_matrix(0,2));
    euler_angles[2] = atan2(-rotation_matrix(0, 1) / cos(euler_angles[1]),
                            rotation_matrix(0, 0) / cos(euler_angles[1]));
    euler_angles[0] = atan2(-rotation_matrix(1, 2) / cos(euler_angles[1]),
                            rotation_matrix(2, 2) / cos(euler_angles[1]));
}

// undistort pc using high-frequency traj
void undistortPointCloud(LidarData raw_pc, vector<PoseData> &pose_vec, gtsam::Pose3 extrinsic, int &pos_index,
                         PointCloudPtr undistort_pc);

// transform rawscan using scan_start_pose and end pose
void transformRawScan(ITPointCloudPtr raw_scan, gtsam::Pose3 scan_start_pose, gtsam::Pose3 scan_end_pose,
                      double scan_start_time, double scan_end_time,
                      PointCloudPtr scan_transformed);

///@brief create directory if does not exist
///
/// \param[in] dir in std::string format
bool EnsureDir(const std::string &dir);

#endif//HELMET_CALIB_COMMON_PT_OPERATIONS_H