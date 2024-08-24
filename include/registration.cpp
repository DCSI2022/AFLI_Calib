/// Common factor for ICP (e.g., point2plane) and module function for scanToMapRegistration
/// Author: Weitong Wu, wwtgeomatics@gmail.com
/// Version 1.0
/// Create Time: 2023/5/9
/// Update Time: 2023/5/28

#include "registration.h"
#include "common_pt_operations.h"
#include "ikd-Tree/ikd_Tree.h"

bool getCorresponce(PointXYZ point_ori, gtsam::Pose3 &point_pose, KD_TREE<PointXYZ> &ikdtree_map, Eigen::Vector3d &norm, double &d)
{
    PointXYZ point_in_map;
    transformPoint(point_ori, point_pose, point_in_map);

    vector<PointXYZ, Eigen::aligned_allocator<PointXYZ>> nearest_points;
    vector<float> point_search_sqdis;
    ikdtree_map.Nearest_Search(point_in_map, 5, nearest_points, point_search_sqdis);

    Eigen::Matrix<double, 5, 3> mat_A0;
    Eigen::Matrix<double, 5, 1> mat_B0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
    if (point_search_sqdis[4] < 9.0) {
        for (int k1 = 0; k1 < 5; ++k1) {
            mat_A0(k1, 0) = nearest_points[k1].x;
            mat_A0(k1, 1) = nearest_points[k1].y;
            mat_A0(k1, 2) = nearest_points[k1].z;
        }
        /// plane ax + by + cz + 1 = 0 -> plane a'x + b'y + c'z + d = 0 and (a',b',c') is normalized
        norm = mat_A0.colPivHouseholderQr().solve(mat_B0);

        if (std::isnan(norm[0]) || std::isnan(norm[1]) || std::isnan(norm[2]) || std::isinf(norm[0]) || std::isinf(norm[1]) || std::isinf(norm[2]))
            return false;

        d = 1 / norm.norm();
        norm.normalize();

        bool plane_valid = true;
        for (int k1 = 0; k1 < 5; ++k1) {
            // if > 0.2, then plane is not fit well
            if (fabs(norm(0) * nearest_points[k1].x +
                     norm(1) * nearest_points[k1].y +
                     norm(2) * nearest_points[k1].z + d) > 0.2) {
                plane_valid = false;
                break;
            }
        }
        return plane_valid;
    }
}

void scanToMapRegi(PointCloudPtr scan_points, KD_TREE<PointXYZ> &kdtree_map,
                   gtsam::Pose3 &scan_pose,
                   int scan2map_iteration, double reject_distance) {
    vector<PointXYZ, Eigen::aligned_allocator<PointXYZ>> nearest_points;
    vector<float> point_search_sqdis;

    //cout << "scan2map_iteration: " << scan2map_iteration << endl;
    //cout << "reject_distance: " << reject_distance << endl;

    for (size_t j = 0; j < scan2map_iteration; j++) {
        gtsam::NonlinearFactorGraph p2plane_graph;
        gtsam::Values values_scanpose;
        values_scanpose.insert(0, scan_pose);

        gtsam::Pose3 tmp_pose;
        tmp_pose = scan_pose;

        for (size_t k = 0; k < scan_points->size(); k++) {
            PointXYZ point_ori = scan_points->points[k];
            PointXYZ point_in_map;

            transformPoint(point_ori, scan_pose, point_in_map);

            kdtree_map.Nearest_Search(point_in_map, 5, nearest_points, point_search_sqdis);

            Eigen::Matrix<double, 5, 3> mat_A0;
            Eigen::Matrix<double, 5, 1> mat_B0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
            if (point_search_sqdis[4] < reject_distance * reject_distance) {
                for (int k1 = 0; k1 < 5; ++k1) {
                    mat_A0(k1, 0) = nearest_points[k1].x;
                    mat_A0(k1, 1) = nearest_points[k1].y;
                    mat_A0(k1, 2) = nearest_points[k1].z;
                }
                /// plane ax + by + cz + 1 = 0 -> plane a'x + b'y + c'z + d = 0 and (a',b',c') is normalized
                Eigen::Vector3d norm = mat_A0.colPivHouseholderQr().solve(mat_B0);

                if (std::isnan(norm[0]) || std::isnan(norm[1]) || std::isnan(norm[2]) || std::isinf(norm[0]) ||
                    std::isinf(norm[1]) || std::isinf(norm[2]))
                    continue;

                double d = 1 / norm.norm();
                norm.normalize();

                bool plane_valid = true;
                for (int k1 = 0; k1 < 5; ++k1) {
                    // if > 0.2, then plane is not fit well
                    if (fabs(norm(0) * nearest_points[k1].x +
                             norm(1) * nearest_points[k1].y +
                             norm(2) * nearest_points[k1].z + d) > 0.2) {
                        plane_valid = false;
                        break;
                    }
                }
                gtsam::Point3 curr_point(point_ori.x, point_ori.y, point_ori.z);
                if (plane_valid) {

                    gtsam::noiseModel::Diagonal::shared_ptr surf_gaussian_model = gtsam::noiseModel::Diagonal::Variances(
                            (gtsam::Vector(1) << 1.0).finished());

                    gtsam::noiseModel::Robust::shared_ptr surf_noise_model = gtsam::noiseModel::Robust::Create(
                            gtsam::noiseModel::mEstimator::Huber::Create(0.1), surf_gaussian_model);

                    LidarPose3PlaneNormFactor point2plane_factor(0, curr_point, norm, d, surf_noise_model);

                    p2plane_graph.push_back(point2plane_factor);
                }
            }
        }
        gtsam::LevenbergMarquardtParams optimizer_params;
        optimizer_params.setLinearSolverType("MULTIFRONTAL_QR");
        optimizer_params.setRelativeErrorTol(1e-4);
        gtsam::LevenbergMarquardtOptimizer optimizer(p2plane_graph, values_scanpose, optimizer_params);
        auto result = optimizer.optimize();
        scan_pose = result.at<gtsam::Pose3>(0);

        auto delta = tmp_pose.inverse() * scan_pose;
        // cout << scan2map_iteration << "th iteration, rotation change:" << delta.rotation().axisAngle().second * 57.3 << endl;
        // cout << scan2map_iteration << "th iteration, translation change:" << delta.translation().norm() * 100 << endl;
        if (delta.rotation().axisAngle().second * 57.3 < 0.05 &&
            delta.translation().norm() * 100 < 1) // 0.05deg and 1cm
            break;
    }
}

void scanToMapRegi(PointCloudPtr scan_points, PointCloudPtr map,
                   pcl::KdTreeFLANN<PointXYZ>::Ptr kdtree_map, gtsam::Pose3 &scan_pose,
                   int scan2map_iteration, double reject_distance)
{
    vector<int> pointSearchInd;
    vector<float> pointSearchSqDis;

    // cout << "scan2map_iteration: " << scan2map_iteration << endl;
    // cout << "reject_distance: " << reject_distance << endl;

    for (size_t j = 0; j < scan2map_iteration; j++)
    {
        gtsam::NonlinearFactorGraph p2plane_graph;
        gtsam::Values values_scanpose;
        values_scanpose.insert(0, scan_pose);

        gtsam::Pose3 tmp_pose;
        tmp_pose = scan_pose;

        for (size_t k = 0; k < scan_points->size(); k++)
        {
            PointXYZ point_ori = scan_points->points[k];
            PointXYZ point_in_map;

            transformPoint(point_ori, scan_pose, point_in_map);

            kdtree_map->nearestKSearch(point_in_map, 5, pointSearchInd, pointSearchSqDis);

            Eigen::Matrix<double, 5, 3> mat_A0;
            Eigen::Matrix<double, 5, 1> mat_B0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
            if (pointSearchSqDis[4] < reject_distance * reject_distance)
            {
                for (int k1 = 0; k1 < 5; ++k1)
                {
                    mat_A0(k1, 0) = map->points[pointSearchInd[k1]].x;
                    mat_A0(k1, 1) = map->points[pointSearchInd[k1]].y;
                    mat_A0(k1, 2) = map->points[pointSearchInd[k1]].z;
                }
                /// plane ax + by + cz + 1 = 0 -> plane a'x + b'y + c'z + d = 0 and (a',b',c') is normalized
                Eigen::Vector3d norm = mat_A0.colPivHouseholderQr().solve(mat_B0);

                if (std::isnan(norm[0]) || std::isnan(norm[1]) || std::isnan(norm[2]) || std::isinf(norm[0]) || std::isinf(norm[1]) || std::isinf(norm[2]))
                    continue;

                double d = 1 / norm.norm();
                norm.normalize();

                bool plane_valid = true;
                for (int k1 = 0; k1 < 5; ++k1)
                {
                    // if > 0.2, then plane is not fit well
                    if (fabs(norm(0) * map->points[pointSearchInd[k1]].x +
                             norm(1) * map->points[pointSearchInd[k1]].y +
                             norm(2) * map->points[pointSearchInd[k1]].z + d) > 0.2)
                    {
                        plane_valid = false;
                        break;
                    }
                }
                gtsam::Point3 curr_point(point_ori.x, point_ori.y, point_ori.z);
                if (plane_valid)
                {

                    gtsam::noiseModel::Diagonal::shared_ptr surf_gaussian_model = gtsam::noiseModel::Diagonal::Variances(
                            (gtsam::Vector(1) << 1.0).finished());

                    gtsam::noiseModel::Robust::shared_ptr surf_noise_model = gtsam::noiseModel::Robust::Create(
                            gtsam::noiseModel::mEstimator::Huber::Create(0.1), surf_gaussian_model);

                    LidarPose3PlaneNormFactor point2plane_factor(0, curr_point, norm, d, surf_noise_model);

                    p2plane_graph.push_back(point2plane_factor);
                }
            }
        }
        gtsam::LevenbergMarquardtParams optimizer_params;
        optimizer_params.setLinearSolverType("MULTIFRONTAL_QR");
        optimizer_params.setRelativeErrorTol(1e-4);
        gtsam::LevenbergMarquardtOptimizer optimizer(p2plane_graph, values_scanpose, optimizer_params);
        auto result = optimizer.optimize();
        scan_pose = result.at<gtsam::Pose3>(0);

        auto delta = tmp_pose.inverse() * scan_pose;
        // cout << scan2map_iteration << "th iteration, rotation change:" << delta.rotation().axisAngle().second * 57.3 << endl;
        // cout << scan2map_iteration << "th iteration, translation change:" << delta.translation().norm() * 100 << endl;
        if (delta.rotation().axisAngle().second * 57.3 < 0.05 && delta.translation().norm() * 100 < 1) // 0.05deg and 1cm
            break;
    }
}

