/// Adapative frame length LiDAR odometry (Refactoring Version)
/// Based on the experiment code in 2022.02 - 2022.08

/// Author: Weitong Wu
/// Contact: wwtgeomatics@gmail.com
/// Date: 2023.04

/// Contribution
/// 1.adaptive frame length considering registration stability and motion state
/// 2.linear continuous-time based frame-to-map registration

/// Reference: Weitong Wu, Jianping Li, Chi Chen, Bisheng Yang,et al.,
/// AFLI-Calib: Robust LiDAR-IMU extrinsic self-calibration based on adaptive frame length LiDAR odometry.
/// ISPRS Jouranl of Photogrammetry and Remote Sensing, 2023.

#include "registration.h"
#include "common_pt_operations.h"
#include "io.h"
#include "sophus/so3.hpp"
#include "types.h"
#include "imu_process.h"
#include <ceres/ceres.h>
#include <gtsam/geometry/Pose3.h>
#include <ikd-Tree/ikd_Tree.h>
#include <iostream>
#include <omp.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>

using namespace std;

#define VoxelSize 0.2 // meter indoor 0.2 outdoor 0.4
#define LENGTH_DT 0.005 // second
#define MINI_FRAME_LENGTH 0.01 // second
#define MAX_FRAME_LENGTH 0.1 // second
#define Debug true

KD_TREE<pcl::PointXYZ> ikdtree(0.5, 0.6, VoxelSize);

int main(int argc, char **argv) {
    cout << "Let's start!" << endl;
    ros::init(argc, argv, "afl_lidarOdometry_node");
    ros::NodeHandle nh;

    double start_time = omp_get_wtime();

    /// 需要输入的参数1
    string rosbag_path = string(argv[1]); // 绝对路径
    int rosbag_name_index = rosbag_path.find_last_of("/");
    cout << "the bag processed is: " << rosbag_path.substr(rosbag_name_index + 1) << endl;

    /// 需要输入的参数2
    int lidar_type = stoi(argv[2]);
    cout << "the lidar type is: ";
    switch (lidar_type) {
        case LIVOX:
            cout << "livox!" << endl;
            break;

        case VELO16:
            cout << "vlp-16!" << endl;
            break;

        case OUST64:
            cout << "ouster!" << endl;
            break;

        case HESAI:
            cout << "hesai!" << endl;
            break;

        default:
            cout << "Error LiDAR type!" << endl;
            return 0;
    }

    /// 需要输入的参数3
    string lidar_topic = string(argv[3]);
    cout << "the lidar topic is: " << lidar_topic << endl;

    /// parameter 4 and 5
    float match_stability_threshold, motion_linearity_threshold;
    match_stability_threshold = stof(argv[4]);
    motion_linearity_threshold = stof(argv[5]);
    cout << "match_stability_threshold: " << match_stability_threshold << endl;
    cout << "motion_linearity_threshold: " << motion_linearity_threshold << endl;

    /// parameter 6 and 7
    float bag_start, bag_dur;
    bag_start = stof(argv[6]);
    bag_dur = stof(argv[7]);
    cout << "bag start: " << bag_start << endl;
    cout << "bag_dur: " << bag_dur << endl;

    /// Create result dir
    string root_dir = rosbag_path.substr(0, rosbag_name_index);
    //cout << "root dir: " << root_dir << endl;
    stringstream result_dir_path;
    double current_time = ros::Time().now().toSec();
    result_dir_path << root_dir << "/LOG_lo_" << setprecision(13) << current_time;
    EnsureDir(result_dir_path.str());

    string frame_length_path = result_dir_path.str() + "/frame_length.txt";
    ofstream ofs_frameLength(frame_length_path);

    /// 载入数据
    cout << "we start load data!" << endl;
    double t_load_start = omp_get_wtime();
    vector<LidarData> lidar_data;
    vector<std::string> topics;
    topics.push_back(lidar_topic);
    loadLidarDataFromRosbag(rosbag_path, lidar_type, 3, topics, bag_start, bag_dur, lidar_data);

    vector<ImuData> imu_data;
    double imu_dt = 0;
    loadImuDataFromRosbag(rosbag_path, imu_data, imu_dt);
    cout << "Load Data time consume: " << omp_get_wtime() - t_load_start << " s" << endl;

    /// Reorganize lidar data
    ITPointCloudPtr all_lidar_data(new ITPointCloud());
    for (int i = 1; i < lidar_data.size(); i++) {
        auto one_lidar_data = lidar_data[i];
        for (int j = 0; j < one_lidar_data.pointcloud.size(); ++j) {
            PointXYZIT pt;
            pt = one_lidar_data.pointcloud.points[j];
            pt.relative_time_ms = one_lidar_data.start_time_s + one_lidar_data.pointcloud[j].relative_time_ms / 1000.0;
            //cout << setprecision(16) << pt.relative_time_ms << " ";
            all_lidar_data->push_back(pt);
        }
    }
    cout << "all lidar point size:" << all_lidar_data->size() << endl;

    /// Start lidar odometry
    cout << "we start adaptive frame length LiDAR odometry!" << endl;

    /// Map Initialization
    PointCloudPtr map(new PointCloud());
    for (int i = 0; i < lidar_data[0].pointcloud.size(); ++i) {
        PointXYZ pt;
        pt.x = lidar_data[0].pointcloud.points[i].x;
        pt.y = lidar_data[0].pointcloud.points[i].y;
        pt.z = lidar_data[0].pointcloud.points[i].z;
        map->push_back(pt);
    }
    cout << "start build kdtree" << std::endl;
    ikdtree.Build(map->points);
    cout << "kdtree build success" << std::endl;

    vector<IPointCloudPtr> vec_frame_transformed;
    vector<IPointCloudPtr> vec_frame_undistorted;
    vector<PoseData> vec_lidarPoses;
    // First frame and pose
    IPointCloudPtr first_frame(new IPointCloud());
    for (int i = 0; i < lidar_data[0].pointcloud.size(); ++i) {
        PointXYZI pt;
        pt.x = lidar_data[0].pointcloud.points[i].x;
        pt.y = lidar_data[0].pointcloud.points[i].y;
        pt.z = lidar_data[0].pointcloud.points[i].z;
        pt.intensity = lidar_data[0].pointcloud.points[i].reflectivity;
        first_frame->points.push_back(pt);
    }
    vec_frame_transformed.push_back(first_frame);
    vec_frame_undistorted.push_back(first_frame); //assume first frame is still
    PoseData first_pose;
    first_pose.timestamp = lidar_data[1].start_time_s;
    first_pose.pose = gtsam::Pose3();
    vec_lidarPoses.push_back(first_pose);

    int point_index = 0;
    int imu_start_index = 0;
    int imu_end_index = 0;
    bool stability_pass = false;
    bool reduce_frameLength = false;
    double adaptive_frame_time = lidar_data[1].start_time_s;
    while (adaptive_frame_time < lidar_data.back().start_time_s) {

        gtsam::Pose3 frame_pose;
        if (vec_lidarPoses.size() != 0)
            frame_pose = vec_lidarPoses.back().pose;

        ITPointCloudPtr adaptiveFrame_points_raw(new ITPointCloud());

        int count = 0;
        double frame_length;

        while (!stability_pass) {

            frame_length = MINI_FRAME_LENGTH + LENGTH_DT * count;
            //cout << "adaptive frame length: " << frame_length << endl;

            /// add points to frame
            for (int j = point_index; j < all_lidar_data->size(); ++j) {
                if (all_lidar_data->points[j].relative_time_ms < adaptive_frame_time + frame_length) {
                    adaptiveFrame_points_raw->push_back(all_lidar_data->points[j]);
                } else {
                    point_index = j;
                    break;
                }
            }
            //cout << "adaptive frame point size: " << adaptiveFrame_points_raw->size() << endl;

            // motion linearity check

            while (frame_length > MINI_FRAME_LENGTH) {

                vector<ImuData> imu_inFrame, imu_inHalfFrame;
                find_imu_data_in_one_scan(adaptive_frame_time, adaptive_frame_time + frame_length, imu_data,
                                          imu_start_index, imu_end_index, imu_inFrame);
                find_imu_data_in_one_scan(adaptive_frame_time, adaptive_frame_time + frame_length / 2, imu_data,
                                          imu_start_index, imu_end_index, imu_inHalfFrame);
                gtsam::Rot3 rotation_frame, rotation_halfFrame;
                rotation_integration(imu_inFrame, imu_dt, rotation_frame);
                rotation_integration(imu_inHalfFrame, imu_dt, rotation_halfFrame);

                auto interpolated_rotation = gtsam::Rot3().expmap(gtsam::Rot3().logmap(rotation_frame) * 0.5);
                double delta_rotation =
                        (rotation_halfFrame.inverse() * interpolated_rotation).axisAngle().second * 57.3;

                if (delta_rotation > motion_linearity_threshold) {

                    cout << "delta rotation: " << delta_rotation << "degree" << endl;

                    reduce_frameLength = true;
                    frame_length = frame_length - LENGTH_DT;

                    adaptiveFrame_points_raw->clear();
                    //cout << "point index: " << point_index << endl;
                    point_index -= 100000;
                    //cout << "point index: " << point_index << endl;
                    if (point_index < 0)
                        point_index = 0;

                    for (int j = point_index; j < all_lidar_data->size(); ++j) {
                        if (all_lidar_data->points[j].relative_time_ms > adaptive_frame_time) {
                            if (all_lidar_data->points[j].relative_time_ms < (adaptive_frame_time + frame_length)) {
                                adaptiveFrame_points_raw->push_back(all_lidar_data->points[j]);
                            } else {
                                point_index = j;
                                break;
                            }
                        }
                    }
                    //cout << "point index: " << point_index << endl;
                } else
                    break;
            }

            /// downsample raw subframe points
            ITPointCloudPtr matching_points_raw(new ITPointCloud());
            voxelGridFilter1<PointXYZIT>(adaptiveFrame_points_raw, VoxelSize, matching_points_raw);

            if (Debug) {
                cout << "adaptive frame point size: " << adaptiveFrame_points_raw->size() << endl;
                cout << "after downsample size: " << matching_points_raw->size() << endl;
            }

            /// linear continuous-time based frame-to-map registration
            double para_rotation_position_last_curr[6];
            Eigen::Map<Eigen::Matrix<double, 6, 1>> rotation_position_last_curr(para_rotation_position_last_curr);
            for (int j = 0; j < 6; ++j) {
                para_rotation_position_last_curr[j] = 0;
            }
            for (int iteration_count = 0; iteration_count < 15; ++iteration_count) {
                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                ceres::Problem::Options problem_options;
                ceres::Problem problem(problem_options);
                problem.AddParameterBlock(para_rotation_position_last_curr, 6);

                for (int j = 0; j < matching_points_raw->points.size(); ++j) {
                    PointXYZIT pt_raw = matching_points_raw->points[j];
                    PointXYZ point_raw1;
                    point_raw1.x = pt_raw.x;
                    point_raw1.y = pt_raw.y;
                    point_raw1.z = pt_raw.z;

                    double ratio = (pt_raw.relative_time_ms - adaptive_frame_time) / frame_length;
                    // cout << "ration: " << ratio << endl;

                    // interpolate
                    gtsam::Rot3 subframe_rotation_toLastframe = gtsam::Rot3().expmap(
                            rotation_position_last_curr.segment<3>(0));
                    gtsam::Pose3 point_pose_toLastframe = gtsam::Pose3(
                            gtsam::Rot3().expmap(gtsam::Rot3().logmap(subframe_rotation_toLastframe) * ratio),
                            rotation_position_last_curr.segment<3>(3));
                    gtsam::Pose3 point_pose = frame_pose * point_pose_toLastframe;

                    // Get correspondence
                    Eigen::Vector3d norm;
                    double d;
                    if (getCorresponce(point_raw1, point_pose, ikdtree, norm, d)) {
                        gtsam::Point3 curr_point(point_raw1.x, point_raw1.y, point_raw1.z);
                        auto factor = Point2PlaneRatioFactor::Create(curr_point, norm, d, ratio,
                                                                     Eigen::Quaterniond(frame_pose.rotation().matrix()),
                                                                     frame_pose.translation());

                        problem.AddResidualBlock(factor, loss_function, para_rotation_position_last_curr);
                    }
                }

                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_SCHUR;
                options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
                options.max_num_iterations = 100;
                options.minimizer_progress_to_stdout = 0;
                options.num_threads = 6;
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);

                // auto delta = rotation_position_last_curr - rotation_position_last_curr1;

                //                        if (delta.segment<3>(0).norm() < 0.1 / 57.3 &&
                //                            delta.segment<3>(0).norm() < 0.05)//5cm 0.1deg
                //                            break;
            }
            //cout << rotation_position_last_curr.transpose() << endl;

            //check matching stability
            Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();

            for (int j = 0; j < matching_points_raw->points.size(); ++j) {
                PointXYZIT pt_raw = matching_points_raw->points[j];
                PointXYZ point_raw1;
                point_raw1.x = pt_raw.x;
                point_raw1.y = pt_raw.y;
                point_raw1.z = pt_raw.z;

                double ratio = (pt_raw.relative_time_ms - adaptive_frame_time) / frame_length;
                // cout << "ration: " << ratio << endl;

                // interpolate
                gtsam::Rot3 subframe_rotation_toLastframe = gtsam::Rot3().expmap(
                        rotation_position_last_curr.segment<3>(0));
                gtsam::Pose3 point_pose_toLastframe = gtsam::Pose3(
                        gtsam::Rot3().expmap(gtsam::Rot3().logmap(subframe_rotation_toLastframe) * ratio),
                        rotation_position_last_curr.segment<3>(3));
                gtsam::Pose3 point_pose = frame_pose * point_pose_toLastframe;

                // Get correspondence
                Eigen::Vector3d norm;
                double d;
                if (getCorresponce(point_raw1, point_pose, ikdtree, norm, d)) {
                    gtsam::Matrix36 Hpose;
                    gtsam::Point3 curr_point(point_raw1.x, point_raw1.y, point_raw1.z);
                    gtsam::Point3 point_in_world = point_pose.transform_from(curr_point, Hpose);
                    gtsam::Vector1 residual;
                    gtsam::Matrix13 Hnorm, Hpoint;
                    residual(0) = gtsam::Point3(norm).dot(point_in_world, Hnorm, Hpoint) + d;
                    J += (Hpoint * Hpose).transpose() * (Hpoint * Hpose);
                }
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigen_solver(J);
            if (eigen_solver.info() != Eigen::Success)
                abort();
            if (Debug)
                cout << "matching stability value : " << eigen_solver.eigenvalues()[0] << endl;

            if (eigen_solver.eigenvalues()[0] < match_stability_threshold && frame_length < MAX_FRAME_LENGTH &&
                !reduce_frameLength) {
                count++;
                continue;
            }

            stability_pass = true;
            reduce_frameLength = false;

            ofs_frameLength << frame_length << endl;
            if (Debug)
                cout << "frame length : " << frame_length * 1000 << "ms" << endl;

            //Update cur_pose
            gtsam::Pose3 curFrame_pose_toLastframe = gtsam::Pose3(
                    gtsam::Rot3().expmap(rotation_position_last_curr.segment<3>(0)),
                    gtsam::Point3(rotation_position_last_curr.segment<3>(3)));
            frame_pose = frame_pose * curFrame_pose_toLastframe;

            /// correct motion distortion,transform to end
            IPointCloudPtr adaptiveFrame_points_corrected(new IPointCloud());
            PointCloudPtr adaptiveFrame_points_corrected1(new PointCloud());
            for (int j = 0; j < adaptiveFrame_points_raw->size(); ++j) {
                PointXYZIT pt_raw = adaptiveFrame_points_raw->points[j];
                PointXYZ point_raw1, point_inStart, point_inEnd;
                point_raw1.x = pt_raw.x;
                point_raw1.y = pt_raw.y;
                point_raw1.z = pt_raw.z;

                double ratio = (pt_raw.relative_time_ms - adaptive_frame_time) / frame_length;
                // interpolate
                gtsam::Pose3 point_pose_toLastframe = gtsam::Pose3().expmap(
                        gtsam::Pose3().logmap(curFrame_pose_toLastframe) * ratio);

                transformPoint(point_raw1, point_pose_toLastframe, point_inStart);
                transformPoint(point_inStart, curFrame_pose_toLastframe.inverse(), point_inEnd);
                PointXYZI pt;
                pt.x = point_inEnd.x;
                pt.y = point_inEnd.y;
                pt.z = point_inEnd.z;
                pt.intensity = adaptiveFrame_points_raw->points[j].reflectivity;
                adaptiveFrame_points_corrected1->points.push_back(point_inEnd);
                adaptiveFrame_points_corrected->points.push_back(pt);
            }

            PointCloudPtr frame_transformed1(new PointCloud());
            IPointCloudPtr frame_transformed(new IPointCloud());
            transformPointcloud(adaptiveFrame_points_corrected1, frame_pose, frame_transformed1);
            transformPointcloud(adaptiveFrame_points_corrected, frame_pose, frame_transformed);
            vec_frame_undistorted.push_back(adaptiveFrame_points_corrected);
            vec_frame_transformed.push_back(frame_transformed);
            auto add_point_size = ikdtree.Add_Points(frame_transformed1->points, true);
            //cout << "kdtree add point size: " << add_point_size << endl;

            //Update frame start time
            adaptive_frame_time += frame_length;

            PoseData cur_pose;
            cur_pose.timestamp = adaptive_frame_time;
            cur_pose.pose = frame_pose;
            vec_lidarPoses.push_back(cur_pose);
        }
        stability_pass = false;
    }

    //Output
    IPointCloudPtr map_result(new IPointCloud());
    int count2 = 0;
    for (int j = 0; j < vec_frame_transformed.size(); ++j) {
        *map_result += *vec_frame_transformed[j];
        if (j % 500 == 0 && j != 0) {
            std::stringstream map_path;
            map_path << result_dir_path.str() << "/map" << count2 << ".pcd";
            if (map_result->size() != 0) {
                pcl::io::savePCDFileBinaryCompressed(map_path.str(), *map_result);
                map_result->clear();
                count2++;
            }
        }
        if (j == vec_frame_transformed.size() - 1) {
            if (map_result->size() != 0) {
                std::stringstream map_path;
                map_path << result_dir_path.str() << "/map" << count2 << ".pcd";
                pcl::io::savePCDFileBinaryCompressed(map_path.str(), *map_result);
                map_result->clear();
            }
        }
    }

    std::stringstream traj_path;
    traj_path << result_dir_path.str() << "/pose.txt";
    writePoseFile(traj_path.str(), vec_lidarPoses);

    //output pcd directory for balm processing
    /*
    std::string pcd_dir = result_dir_path.str() + "/pcd";
    EnsureDir(pcd_dir);
    for (size_t i = 0; i < vec_frame_undistorted.size(); i++)
    {
        std::stringstream pcd_name;
        pcd_name << pcd_dir << "/" << i << ".pcd";
        pcl::io::savePCDFileBinaryCompressed(pcd_name.str(), *vec_frame_undistorted[i]);
    }
    */
    
    double end_time = omp_get_wtime();
    cout << "run time: " << end_time - start_time << "s" << endl;

    return 0;
}
