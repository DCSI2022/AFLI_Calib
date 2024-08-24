/// Tightly-coupled LiDAR-IMU extrinsic calibration (Refactoring Version)
/// Based on the experiment code in 2022.02 - 2022.08

/// Author: Weitong Wu
/// Contact: wwtgeomatics@gmail.com
/// Date: 2023.05

/// Reference: Weitong Wu, Jianping Li, Chi Chen, Bisheng Yang,et al.,
/// AFLI-Calib: Robust LiDAR-IMU extrinsic self-calibration based on adaptive frame length LiDAR odometry.
/// ISPRS Jouranl of Photogrammetry and Remote Sensing, 2023.

#include <Eigen/Eigen>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <iostream>
#include <omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include "common_pt_operations.h"
#include "imu_process.h"
#include "io.h"
#include "types.h"
#include "handeye.h"
#include "registration.h"

#define MapVoxelSize 0.4
#define ScanVoxelSize 0.2
#define LidarWeight 1

using gtsam::symbol_shorthand::B;// IMU Bias
using gtsam::symbol_shorthand::E;// Extrinsic from lidar and imu
using gtsam::symbol_shorthand::V;// Velocity
using gtsam::symbol_shorthand::X;// Pose3

using namespace std;

KD_TREE<pcl::PointXYZ> ikdtree(0.5, 0.6, MapVoxelSize);

int main(int argc, char **argv) {

    cout << "start tightly licalib! " << endl;
    ros::init(argc, argv, "tight_licalib_node");
    ros::NodeHandle nh;

    /// 需要输入的参数1
    string rosbag_path = string(argv[1]); // 绝对路径
    int rosbag_name_index = rosbag_path.find_last_of("/");
    cout << "the bag processed is: " << rosbag_path.substr(rosbag_name_index + 1) << endl;

    /// 需要输入的参数2
    string lo_pose_path = string(argv[2]);

    /// 需要输入的参数3
    int lidar_type = stoi(argv[3]);
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

    /// 需要输入的参数4
    string lidar_topic = string(argv[4]);
    cout << "the lidar topic is: " << lidar_topic << endl;

    /// 需要输入的参数5,6
    float bag_start, bag_dur;
    bag_start = stof(argv[5]);
    bag_dur = stof(argv[6]);
    cout << "bag start: " << bag_start << endl;
    cout << "bag_dur: " << bag_dur << endl;

    /// 需要输入的参数7
    int still_time;
    still_time = stoi(argv[7]);
    cout << "still time: " << still_time << endl;

    /// Create result dir
    string root_dir = rosbag_path.substr(0, rosbag_name_index);
    //cout << "root dir: " << root_dir << endl;
    stringstream result_dir_path;
    double current_time = ros::Time().now().toSec();
    result_dir_path << root_dir << "/LOG_calib_" << setprecision(13) << current_time;
    EnsureDir(result_dir_path.str());

    // 1. load lidar pose by afl_lidarOdometry
    vector<PoseData> lo_poses;
    loadPoseFile(lo_pose_path, lo_poses);

    // 2. load rosbag information
    vector<ImuData> imu_data;
    double imu_dt;
    int point_filter_num = 3;
    loadImuDataFromRosbag(rosbag_path, imu_data, imu_dt);
    vector<LidarData> lidar_data;
    vector<std::string> topics;
    topics.push_back(lidar_topic);
    loadLidarDataFromRosbag(rosbag_path, lidar_type, point_filter_num, topics, bag_start, bag_dur, lidar_data);

    /// Reorganize lidar data
    ITPointCloudPtr all_lidar_data(new ITPointCloud());
    for (int i = 1; i < lidar_data.size(); i++) {
        auto one_lidar_data = lidar_data[i];
        for (int j = 0; j < one_lidar_data.pointcloud.size(); ++j) {
            PointXYZIT pt;
            pt = one_lidar_data.pointcloud[j];
            pt.relative_time_ms = one_lidar_data.start_time_s + one_lidar_data.pointcloud[j].relative_time_ms / 1000.0;
            //cout << setprecision(16) << pt.relative_time_ms << " ";
            all_lidar_data->push_back(pt);
        }
    }
    cout << "all lidar point size:" << all_lidar_data->size() << endl;

    // 3. get the corresponding lidar and imu data in one scan
    vector<CalibData> calib_data;
    int imu_data_start_index = 0;
    int imu_data_end_index = 0;
    int point_index = 0;
    for (int j = 0; j < lo_poses.size() - 1; ++j) {
        CalibData one_segment;
        one_segment.scan_start_time = lo_poses[j].timestamp;
        one_segment.scan_end_time = lo_poses[j + 1].timestamp;
        one_segment.scan_start_pose = lo_poses[j].pose;
        one_segment.scan_end_pose = lo_poses[j + 1].pose;

        vector<ImuData> one_segment_imu;
        find_imu_data_in_one_scan(one_segment.scan_start_time, one_segment.scan_end_time,
                                  imu_data, imu_data_start_index, imu_data_end_index, one_segment_imu);
        one_segment.imu_data = one_segment_imu;

        for (int k = point_index; k < all_lidar_data->size(); ++k) {
            if (all_lidar_data->points[k].relative_time_ms > one_segment.scan_start_time) {
                if (all_lidar_data->points[k].relative_time_ms < one_segment.scan_end_time) {
                    one_segment.raw_scan.push_back(all_lidar_data->points[k]);
                } else {
                    point_index = k;
                    break;
                }
            }

        }
        /*
        cout << "scan start time, end time, imu start time, end time, point start time, point end time: "
             << setprecision(13) <<
             one_segment.scan_start_time << " " << one_segment.scan_end_time << " " <<
             one_segment_imu.front().timestamp_s << " " <<
             one_segment_imu.back().timestamp_s << " " << one_segment.raw_scan.front().relative_time_ms << " " <<
             one_segment.raw_scan.back().relative_time_ms << endl;
             */
        calib_data.push_back(one_segment);
    }
    // IMU pre-integration
    auto preInte_parameter = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(9.7936);
    preInte_parameter->accelerometerCovariance = gtsam::I_3x3 * pow(0.1, 2);
    preInte_parameter->gyroscopeCovariance = gtsam::I_3x3 * pow(0.001, 2);
    preInte_parameter->biasAccCovariance = gtsam::I_3x3 * pow(0.0001, 2);
    preInte_parameter->biasOmegaCovariance = gtsam::I_3x3 * pow(0.0001, 2);
    preInte_parameter->integrationCovariance =
            gtsam::I_3x3 * 1e-8;// error committed in integrating position from velocities
    preInte_parameter->biasAccOmegaInt = gtsam::I_6x6 * 1e-5;   // error in the bias used for preintegration
    auto preintegrated = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(preInte_parameter,
                                                                                    gtsam::imuBias::ConstantBias());

    vector<gtsam::PreintegratedCombinedMeasurements> vec_preintegration;
    for (int j = 0; j < calib_data.size(); ++j) {
        preintegrated->resetIntegrationAndSetBias(gtsam::imuBias::ConstantBias());
        for (int k = 0; k < calib_data[j].imu_data.size(); ++k) {
            preintegrated->integrateMeasurement(calib_data[j].imu_data[k].am, calib_data[j].imu_data[k].wm, imu_dt);
        }
        vec_preintegration.push_back(*preintegrated);
    }

    // 4. Initialize rotation extrinsic
    vector<gtsam::Rot3> vec_lidar_relaRota, vec_imu_relaRota;
    for (int j = 0; j < calib_data.size(); ++j) {
        vec_lidar_relaRota.push_back(
                (calib_data[j].scan_start_pose.inverse() * calib_data[j].scan_end_pose).rotation());
    }
    for (int j = 0; j < vec_preintegration.size(); ++j) {
        vec_imu_relaRota.push_back(vec_preintegration[j].deltaRij());
    }
    Eigen::Matrix3d R_LtoI;
    hand_eye_calibration_rotation(vec_imu_relaRota, vec_lidar_relaRota, R_LtoI);

    gtsam::Point3 p_LtoI(0, 0, 0);
    gtsam::Pose3 T_LtoI = gtsam::Pose3(gtsam::Rot3(R_LtoI), p_LtoI);

    // 5. state initialization
    vector<ImuData> still_imu_data;
    for (int j = 0; j < imu_data.size(); ++j) {
        if ((imu_data[j].timestamp_s - imu_data.front().timestamp_s) < still_time)
            still_imu_data.push_back(imu_data[j]);
        else
            break;
    }
    std::cout << "still_imu_data size:" << still_imu_data.size() << std::endl;

    gtsam::Pose3 T_I0toG;
    Eigen::Vector3d b_w0;
    Eigen::Vector3d b_a0;

    imu_initialization(still_imu_data, T_I0toG, b_w0, b_a0);

    // 6. iteration: lidar factor construction  + optimization
    gtsam::Values optimized_states;
    int iter_num = 3;
    for (int j = 0; j < iter_num; ++j) {

        ///Set up the factor graph
        std::shared_ptr<gtsam::NonlinearFactorGraph> calibration_factor_graph =
                std::make_shared<gtsam::NonlinearFactorGraph>();
        std::shared_ptr<gtsam::Values> calibration_initial_values =
                std::make_shared<gtsam::Values>();

        /// Add all prior factors to the graph
        auto prior_pose_noise_model = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 0.002, 0.002, 1.0e-5, 1.0e-05, 1.0e-05, 1.0e-05).finished());// rad,rad,rad,m, m, m
        auto prior_velocity_noise_model = gtsam::noiseModel::Isotropic::Sigma(3, 1.0e-5);       // m/s

        gtsam::Vector6 prior_biasSigmas;
        prior_biasSigmas.head<3>().setConstant(0.1);
        prior_biasSigmas.tail<3>().setConstant(0.01);
        gtsam::SharedNoiseModel prior_bias_noise_model = gtsam::noiseModel::Diagonal::Sigmas(prior_biasSigmas);

        gtsam::imuBias::ConstantBias prior_imu_bias = gtsam::imuBias::ConstantBias(b_a0, b_w0);

        calibration_factor_graph->addPrior<gtsam::Pose3>(X(0), T_I0toG, prior_pose_noise_model);
        calibration_factor_graph->addPrior<gtsam::Vector3>(V(0), gtsam::Point3(), prior_velocity_noise_model);
        calibration_factor_graph->addPrior<gtsam::imuBias::ConstantBias>(B(0), prior_imu_bias, prior_bias_noise_model);

        calibration_initial_values->insert(X(0), T_I0toG);
        calibration_initial_values->insert(V(0), gtsam::Vector3(0, 0, 0));
        calibration_initial_values->insert(B(0), prior_imu_bias);
        calibration_initial_values->insert(E(0), T_LtoI);

        int state_index = 0;

        PointCloudPtr map(new PointCloud());
        if (j == 0) {
            for (int k = 0; k < calib_data.size(); ++k) {
                //using scan_start_pose and scan_end_pose get transformed scan
                PointCloudPtr scan_transformed(new PointCloud());
                ITPointCloudPtr raw_scan(new ITPointCloud());
                *raw_scan = calib_data[k].raw_scan;
                transformRawScan(raw_scan, calib_data[k].scan_start_pose, calib_data[k].scan_end_pose,
                                 calib_data[k].scan_start_time, calib_data[k].scan_end_time, scan_transformed);
                *map += *scan_transformed;
            }
        } else {
            for (int k = 0; k < calib_data.size(); ++k) {
                if ((calib_data[k].scan_start_time - calib_data.front().scan_start_time) > still_time) {
                    //using scan_start_pose and scan_end_pose get transformed scan
                    PointCloudPtr scan_transformed(new PointCloud());
                    ITPointCloudPtr raw_scan(new ITPointCloud());
                    *raw_scan = calib_data[k].raw_scan;
                    transformRawScan(raw_scan, calib_data[k].scan_start_pose_fused, calib_data[k].scan_end_pose_fused,
                                     calib_data[k].scan_start_time, calib_data[k].scan_end_time, scan_transformed);
                    *map += *scan_transformed;
                }
            }
        }
        ikdtree.Build(map->points);

        int calib_segment_index = 0;
        for (int k = 0; k < calib_data.size(); ++k) {
            if ((calib_data[k].scan_start_time - calib_data.front().scan_start_time) > still_time) {

                if (calib_segment_index == 0) {
                    calib_segment_index = k;
                    cout << "calib_segment_index: " << calib_segment_index << endl;
                }

                ITPointCloudPtr rawscan_downsample(new ITPointCloud());
                ITPointCloudPtr rawscan(new ITPointCloud());
                *rawscan = calib_data[k].raw_scan;
                voxelGridFilter1<PointXYZIT>(rawscan, ScanVoxelSize, rawscan_downsample);

                //Add LiDAR factor
                for (int l = 0; l < rawscan_downsample->size(); ++l) {
                    PointXYZIT pt_raw = rawscan_downsample->points[l];
                    PointXYZ point_raw1;
                    point_raw1.x = pt_raw.x;
                    point_raw1.y = pt_raw.y;
                    point_raw1.z = pt_raw.z;

                    double ratio = (pt_raw.relative_time_ms - calib_data[k].scan_start_time)
                                   / (calib_data[k].scan_end_time - calib_data[k].scan_start_time);
                    // cout << "ration: " << ratio << endl;

                    // interpolate
                    gtsam::Pose3 point_pose = calib_data[k].scan_start_pose * gtsam::Pose3().expmap(
                            gtsam::Pose3().logmap(
                                    calib_data[k].scan_start_pose.inverse() * calib_data[k].scan_end_pose) * ratio);

                    // Get correspondence
                    Eigen::Vector3d norm;
                    double d;
                    if (getCorresponce(point_raw1, point_pose, ikdtree, norm, d)) {
                        gtsam::Point3 curr_point(point_raw1.x, point_raw1.y, point_raw1.z);

                        gtsam::noiseModel::Diagonal::shared_ptr planar_gaussian_model = gtsam::noiseModel::Diagonal::Variances(
                                (gtsam::Vector(1) << 0.05).finished());
                        gtsam::noiseModel::Robust::shared_ptr planar_noise_model = gtsam::noiseModel::Robust::Create(
                                gtsam::noiseModel::mEstimator::Huber::Create(LidarWeight * 0.1 / sqrt(0.05)),
                                planar_gaussian_model);
                        LidarPlaneNormwithExtrinsictoIMUFactor lidar_factor(X(state_index + 1), E(0), curr_point, norm,
                                                                            d, LidarWeight, T_I0toG,
                                                                            planar_noise_model);
                        calibration_factor_graph->push_back(lidar_factor);
                    }
                }

                //Add IMU factor
                if (j == 0)
                    preintegrated->resetIntegrationAndSetBias(prior_imu_bias);
                else
                    preintegrated->resetIntegrationAndSetBias(
                            optimized_states.at<gtsam::imuBias::ConstantBias>(B(state_index)));

                for (int l = 0; l < calib_data[k].imu_data.size(); ++l) {
                    preintegrated->integrateMeasurement(calib_data[k].imu_data[l].am, calib_data[k].imu_data[l].wm,
                                                        imu_dt);
                }
                auto preint_imu_combined = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements &>(*preintegrated);
                gtsam::CombinedImuFactor imuFactor(X(state_index), V(state_index), X(state_index + 1),
                                                   V(state_index + 1),
                                                   B(state_index), B(state_index + 1), preint_imu_combined);

                calibration_factor_graph->add(imuFactor);

                state_index++;

                ///Add initial value
                if (j == 0) {
                    auto current_imu_pose = T_I0toG * (T_LtoI * calib_data[k].scan_end_pose * T_LtoI.inverse());
                    gtsam::Pose3 last_imu_pose = T_I0toG * (T_LtoI * calib_data[k].scan_start_pose * T_LtoI.inverse());

                    Eigen::Vector3d current_velocity = (current_imu_pose.translation() - last_imu_pose.translation()) /
                                                       (calib_data[k].scan_end_time - calib_data[k].scan_start_time);

                    calibration_initial_values->insert(X(state_index), current_imu_pose);
                    calibration_initial_values->insert(V(state_index), current_velocity);
                    calibration_initial_values->insert(B(state_index), prior_imu_bias);
                } else {

                    calibration_initial_values->insert(X(state_index),
                                                       optimized_states.at<gtsam::Pose3>(X(state_index)));
                    calibration_initial_values->insert(V(state_index),
                                                       optimized_states.at<gtsam::Vector3>(V(state_index)));
                    calibration_initial_values->insert(B(state_index),
                                                       optimized_states.at<gtsam::imuBias::ConstantBias>(
                                                               B(state_index)));
                }
            }
        }

        ///Optimization
        std::cout << "start optimization..." << std::endl;
        double t_optimize;
        t_optimize = omp_get_wtime();
        gtsam::LevenbergMarquardtParams lm_opt_parameters;
        lm_opt_parameters.setLinearSolverType("MULTIFRONTAL_CHOLESKY");
        lm_opt_parameters.setRelativeErrorTol(1e-4);

        gtsam::LevenbergMarquardtOptimizer lm_optimizer(*calibration_factor_graph, *calibration_initial_values,
                                                        lm_opt_parameters);
        optimized_states = lm_optimizer.optimize();
        std::cout << "tightly coupled optimization time: " << omp_get_wtime() - t_optimize << "s" << std::endl;

        //debug
        //    calibration_factor_graph->print();
        //    calibration_initial_values->print();
        //    result.print();
        T_LtoI = optimized_states.at<gtsam::Pose3>(E(0));
        prior_imu_bias = optimized_states.at<gtsam::imuBias::ConstantBias>(B(0));

        // update poses
        for (int k = 0; k < state_index - 1; ++k) {
            calib_data[k + calib_segment_index].scan_start_pose_fused = T_LtoI.inverse() * T_I0toG.inverse() *
                                                                        optimized_states.at<gtsam::Pose3>(X(k)) *
                                                                        T_LtoI;
            calib_data[k + calib_segment_index].scan_end_pose_fused = T_LtoI.inverse() * T_I0toG.inverse() *
                                                                      optimized_states.at<gtsam::Pose3>(X(k + 1)) *
                                                                      T_LtoI;
        }

        ///Output
        Eigen::Vector3d optimized_euler_angle;
        rotationMatrix_to_eulerAngles(T_LtoI.rotation().matrix(), optimized_euler_angle);
        std::cout << "optimized translation: " << T_LtoI.translation().transpose() << std::endl;
        std::cout << "optimized rotation in yaw pitch roll: " << optimized_euler_angle.transpose() << std::endl;

        std::stringstream extrinsic_path;
        extrinsic_path << result_dir_path.str() << "/estimated_extrinsic.txt";
        std::ofstream ofs_extrinsic(extrinsic_path.str());
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                ofs_extrinsic << T_LtoI.rotation().matrix()(j, k) << " ";
            }
            ofs_extrinsic << T_LtoI.translation()[j] << std::endl;
        }
        ofs_extrinsic.close();
    }
    return 0;
}