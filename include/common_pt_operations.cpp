/// Common point operations and other functions
/// Author: Weitong Wu, wwtgeomatics@gmail.com
/// Version 1.1
/// Create Time: 2022/5/23
/// Update Time: 2023/5/28

#include "common_pt_operations.h"

void undistortPointCloud(LidarData raw_pc, vector<PoseData> &pose_vec, gtsam::Pose3 extrinsic, int &pos_index,
                         PointCloudPtr undistort_pc) {
    gtsam::Pose3 scan_end_pose;
    PointCloudPtr tmp_pc(new PointCloud());
    for (int i = 0; i < raw_pc.pointcloud.size(); ++i) {
        double point_time = raw_pc.start_time_s +
                            raw_pc.pointcloud.points[i].relative_time_ms / 1000.0;
        double point_time1 = calUTCWeekSec(point_time);
        for (int j = pos_index; j < pose_vec.size() - 1; ++j) {
            if (point_time1 > pose_vec[j].timestamp &&
                point_time1 < pose_vec[j + 1].timestamp) {

                double ratio =
                        (point_time1 - pose_vec[j].timestamp) / (pose_vec[j + 1].timestamp - pose_vec[j].timestamp);

                // interpolate
                gtsam::Pose3 point_pose = pose_vec[j].pose * gtsam::Pose3().expmap(
                        gtsam::Pose3().logmap(pose_vec[j].pose.inverse() * pose_vec[j + 1].pose) * ratio);
                if (i == raw_pc.pointcloud.size() - 1)
                    scan_end_pose = point_pose;
                PointXYZ pt, pt_transformed;
                pt.x = raw_pc.pointcloud.points[i].x;
                pt.y = raw_pc.pointcloud.points[i].y;
                pt.z = raw_pc.pointcloud.points[i].z;
                transformPoint(pt, point_pose * extrinsic, pt_transformed);
                tmp_pc->points.push_back(pt_transformed);
                pos_index = j;
                break;
            }
        }
    }
    // transform pc to scan_start coordinate
    transformPointcloud(tmp_pc, extrinsic.inverse() * scan_end_pose.inverse(), undistort_pc);
}

void transformRawScan(ITPointCloudPtr raw_scan, gtsam::Pose3 scan_start_pose, gtsam::Pose3 scan_end_pose,
                      double scan_start_time, double scan_end_time,
                      PointCloudPtr scan_transformed) {
    for (int i = 0; i < raw_scan->size(); ++i) {
        PointXYZIT pt_raw = raw_scan->points[i];
        PointXYZ point_raw1, point_transformed;
        point_raw1.x = pt_raw.x;
        point_raw1.y = pt_raw.y;
        point_raw1.z = pt_raw.z;

        double ratio = (pt_raw.relative_time_ms - scan_start_time) / (scan_end_time - scan_start_time);

        // interpolate
        gtsam::Pose3 point_pose = scan_start_pose * gtsam::Pose3().expmap(
                gtsam::Pose3().logmap(
                        scan_start_pose.inverse() * scan_end_pose) * ratio);

        transformPoint(point_raw1, point_pose, point_transformed);
        scan_transformed->points.push_back(point_transformed);
    }
}

///@brief create directory if does not exist
///
/// \param[in] dir in std::string format
bool EnsureDir(const std::string &dir) {
    if (dir == "") {
        return false;
    }
    bool bSuccess = true;
    boost::filesystem::path fullpath(dir);
    boost::filesystem::path parent_path = fullpath.parent_path();
    if (!boost::filesystem::exists(parent_path)) {
        bSuccess |= EnsureDir(parent_path.string());
        bSuccess |= boost::filesystem::create_directory(fullpath);
    } else if (!boost::filesystem::exists(fullpath)) {
        bSuccess |= boost::filesystem::create_directory(fullpath);
    }
    return bSuccess;
}

