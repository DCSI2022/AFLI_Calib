/// Common factor for ICP (e.g., point2plane) and module function for scanToMapRegistration
/// Author: Weitong Wu, wwtgeomatics@gmail.com
/// Version 1.0
/// Create Time: 2023/5/9
/// Update Time: 2023/5/28

#ifndef SRC_REGISTRATION_H
#define SRC_REGISTRATION_H

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Pose3.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "types.h"
#include "ikd-Tree/ikd_Tree.h"
#include "sophus/so3.hpp"
#include <ceres/ceres.h>

// point2plane factor in GTSAM
class LidarPose3PlaneNormFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
private:
    gtsam::Point3 curr_point_;
    gtsam::Point3 plane_unit_norm_;
    double d_;

public:
    LidarPose3PlaneNormFactor(gtsam::Key pose_key, gtsam::Point3 curr_point,
                              gtsam::Point3 plane_unit_norm, double d,
                              gtsam::SharedNoiseModel noise_model)
            : gtsam::NoiseModelFactor1<gtsam::Pose3>(noise_model, pose_key),
              curr_point_(curr_point), plane_unit_norm_(plane_unit_norm), d_(d) {}

    gtsam::Vector evaluateError(const gtsam::Pose3 &pose, boost::optional<gtsam::Matrix &> H1 = boost::none) const {

        gtsam::Matrix36 Hpose;
        gtsam::Point3 point_in_world = pose.transform_from(curr_point_, H1 ? &Hpose : 0);

        gtsam::Vector1 residual;
        gtsam::Matrix13 Hnorm, Hpoint;
        residual(0) = plane_unit_norm_.dot(point_in_world, Hnorm, Hpoint) + d_;

        if (H1) {
            H1->resize(1, 6);

            *H1 << Hpoint * Hpose;
        }

        return residual;
    }
};

// linear-based continuous time point2plane factor in Ceres
struct Point2PlaneRatioFactor {
    Point2PlaneRatioFactor(Eigen::Vector3d curr_point, Eigen::Vector3d plane_unit_norm,
                           double d, double ratio,
                           Eigen::Quaterniond q_last, Eigen::Vector3d t_last) {
        curr_point_ = curr_point;
        plane_unit_norm_ = plane_unit_norm;
        d_ = d;
        ratio_ = ratio;
        q_last_ = q_last;
        t_last_ = t_last;
    }

    template<typename T>
    bool operator()(const T *RP, T *residual) const {

        Eigen::Map<const Eigen::Matrix<T, 6, 1>> rotation_position_vector(RP);
        Eigen::Quaternion<T> q_last_curr = Sophus::SO3<T>::exp(
                rotation_position_vector.template segment<3>(0)).unit_quaternion();
        Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
        Eigen::Quaternion<T> q_last_point = q_identity.slerp(T(ratio_), q_last_curr);
        Eigen::Quaternion<T> q_world_last{T(q_last_.w()), T(q_last_.x()), T(q_last_.y()), T(q_last_.z())};
        Eigen::Matrix<T, 3, 1> t_last_point{rotation_position_vector[3], rotation_position_vector[4],
                                            rotation_position_vector[5]};
        Eigen::Matrix<T, 3, 1> t_world_last{T(t_last_.x()), T(t_last_.y()), T(t_last_.z())};

        Eigen::Matrix<T, 3, 1> point_body{T(curr_point_.x()), T(curr_point_.y()), T(curr_point_.z())};
        Eigen::Matrix<T, 3, 1> point_world = q_world_last * (q_last_point * point_body + t_last_point) + t_world_last;

        Eigen::Matrix<T, 3, 1> norm{T(plane_unit_norm_.x()), T(plane_unit_norm_.y()), T(plane_unit_norm_.z())};
        residual[0] = norm.dot(point_world) + T(d_);

        return true;
    }

    static ceres::CostFunction *Create(Eigen::Vector3d curr_point, Eigen::Vector3d plane_unit_norm,
                                       double d, double ratio,
                                       Eigen::Quaterniond q_last, Eigen::Vector3d t_last) {
        return (new ceres::AutoDiffCostFunction<Point2PlaneRatioFactor, 1, 6>(
                new Point2PlaneRatioFactor(curr_point, plane_unit_norm, d, ratio, q_last, t_last)));
    }

private:
    Eigen::Vector3d curr_point_;
    Eigen::Vector3d plane_unit_norm_;
    double d_, ratio_;
    Eigen::Quaterniond q_last_;
    Eigen::Vector3d t_last_;
};

// Todo check the Jacobian
class LidarPlaneNormwithExtrinsictoIMUFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
private:
    gtsam::Point3 curr_point_, plane_unit_norm_;
    gtsam::Pose3 pose_I0toG_;
    double d_, w_;

public:
    LidarPlaneNormwithExtrinsictoIMUFactor(gtsam::Key pose_ItoG_key, gtsam::Key pose_LtoI_key, gtsam::Point3 curr_point,
                                           gtsam::Point3 plane_unit_norm, double d, double w, gtsam::Pose3 pose_I0toG,
                                           gtsam::SharedNoiseModel noise_model)
            : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(noise_model, pose_ItoG_key, pose_LtoI_key),
              curr_point_(curr_point), plane_unit_norm_(plane_unit_norm), d_(d), w_(w), pose_I0toG_(pose_I0toG){}

    gtsam::Vector evaluateError(const gtsam::Pose3 &pose_ItoG, const gtsam::Pose3 &pose_LtoI,
                                const boost::optional<gtsam::Matrix &> H1 = boost::none,
                                const boost::optional<gtsam::Matrix &> H2 = boost::none) const {
        gtsam::Point3 point_in_L0 = pose_LtoI.inverse() * pose_I0toG_.inverse() * pose_ItoG * pose_LtoI * curr_point_;

        gtsam::Matrix13 Hnorm, Hpoint;

        gtsam::Vector1 residual;

        residual(0) = w_ * (plane_unit_norm_.dot(point_in_L0, Hnorm, Hpoint) + d_);

        if (H1) {
            H1->resize(1, 6);

            auto tmp1 =  pose_LtoI.inverse() * pose_I0toG_.inverse() * pose_ItoG;
            auto tmp2 = pose_LtoI * curr_point_;

            gtsam::Matrix36 Hpose;
            Hpose.block<3, 3>(0, 0) = tmp1.rotation().matrix() * gtsam::skewSymmetric(-tmp2.x(), -tmp2.y(), -tmp2.z());
            Hpose.block<3, 3>(0, 3) = tmp1.rotation().matrix();

            *H1 << Hpoint * Hpose;
        }

        if (H2) {
            H2->resize(1, 6);

            auto tmp1 = pose_LtoI.inverse() * pose_I0toG_.inverse() * pose_ItoG * pose_LtoI;
            auto tmp2 = pose_LtoI.inverse() * pose_I0toG_.inverse() * pose_ItoG * pose_LtoI * curr_point_;

            gtsam::Matrix36 H_extrinsic;

            H_extrinsic.block<3, 3>(0, 0) = tmp1.rotation().matrix() *
                                            gtsam::skewSymmetric(-curr_point_.x(), -curr_point_.y(), -curr_point_.z()) -
                                            gtsam::skewSymmetric(-tmp2.x(), -tmp2.y(), -tmp2.z());
            H_extrinsic.block<3, 3>(0, 3) = tmp1.rotation().matrix();

            *H2 << Hpoint * H_extrinsic;
        }

        return residual;
    }
};

// given raw point and point pose to find LOAM plane correspondence in map using ikdtree
bool getCorresponce(PointXYZ point_ori, gtsam::Pose3 &point_pose, KD_TREE<PointXYZ> &ikdtree_map,
                    Eigen::Vector3d &norm, double &d);

// Using ikdtree
void scanToMapRegi(PointCloudPtr scan_points, KD_TREE<PointXYZ> &kdtree_map,
                   gtsam::Pose3 &scan_pose,
                   int scan2map_iteration = 15, double reject_distance = 3);

// Using pcl kdtree
void scanToMapRegi(PointCloudPtr scan_points, PointCloudPtr map,
                   pcl::KdTreeFLANN<PointXYZ>::Ptr kdtree_map, gtsam::Pose3 &scan_pose,
                   int scan2map_iteration = 15, double reject_distance = 3);

#endif //SRC_REGISTRATION_H
