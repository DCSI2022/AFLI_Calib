/// hand eye calibration
/// Author: Weitong Wu, wwtgeomatics@gmail.com
/// Version 1.1
/// Create Time: 2021
/// Update Time: 2023/5/28

#include "handeye.h"
#include "common_pt_operations.h"

using namespace std;
using namespace gtsam;

/*
 * @Input: 2 sensor trajectory (Quaternion and position) with time
 * @Output: extrinsic: r_2To1, p_2To1
 */

void hand_eye_calibration(vector<Eigen::Quaterniond> &sensor1_q_vec, vector<Eigen::Vector3d> &sensor1_p_vec, vector<double> &sensor1_time_vec,
                          vector<Eigen::Quaterniond> &sensor2_q_vec, vector<Eigen::Vector3d> &sensor2_p_vec, vector<double> &sensor2_time_vec,
                          Eigen::Matrix3d &r_2To1, Eigen::Vector3d &p_2To1)
{
    /// Reference: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=326576
    /// https://arxiv.org/pdf/2001.06175.pdf(formula is slightly wrong)
    /// http://campar.in.tum.de/view/Chair/HandEyeCalibration

    /// First： get relative motion pair for each sensor {qTolast,pTolast}
    vector<Eigen::Quaterniond> sensor1_qTolast_vec, sensor2_qTolast_vec;
    vector<Eigen::Vector3d> sensor1_pTolast_vec, sensor2_pTolast_vec;

    int imu1_start_index, imu1_end_index;
    for (int i = 0; i < sensor2_q_vec.size()-1; ++i) {
        imu1_start_index = -1;
        imu1_end_index = -1;
        for (int j = 0; j < sensor1_q_vec.size(); ++j) {
            if (fabs(sensor1_time_vec[j] - sensor2_time_vec[i]) < 0.001)
            {
                imu1_start_index = j;
                break;
            }
        }
        if (imu1_start_index == -1)
            continue;
        for (int j = imu1_start_index; j < sensor1_q_vec.size() ; ++j) {
            if (fabs(sensor1_time_vec[j] - sensor2_time_vec[i+1]) < 0.001)
            {
                imu1_end_index = j;
                break;
            }
        }
        if (imu1_end_index == -1)
            continue;

        Eigen::Quaterniond imu1_qTolast, imu2_qTolast;
        Eigen::Vector3d imu1_pTolast, imu2_pTolast;
        //Todo write as function
        imu1_qTolast = sensor1_q_vec[imu1_start_index].inverse() * sensor1_q_vec[imu1_end_index];
        imu1_pTolast = sensor1_q_vec[imu1_start_index].inverse() * (sensor1_p_vec[imu1_end_index] - sensor1_p_vec[imu1_start_index]);
        imu2_qTolast = sensor2_q_vec[i].inverse() * sensor2_q_vec[i+1];
        imu2_pTolast = sensor2_q_vec[i].inverse() * (sensor2_p_vec[i+1] - sensor2_p_vec[i]);
        sensor1_qTolast_vec.push_back(imu1_qTolast);
        sensor1_pTolast_vec.push_back(imu1_pTolast);
        sensor2_qTolast_vec.push_back(imu2_qTolast);
        sensor2_pTolast_vec.push_back(imu2_pTolast);
    }

    int motionPair_num = sensor1_qTolast_vec.size();
    cout << "relative motion pair num: " << motionPair_num << std::endl;

    /// Second： calculate extrinsic of rotation
    Eigen::Matrix3d covariance_matrix = Eigen::Matrix3d::Zero();
    for (int i = 0; i < motionPair_num; ++i) {
        Eigen::Vector3d imu1_qTolast_so3  = Rot3::Logmap(Rot3(sensor1_qTolast_vec[i]));
        Eigen::Vector3d imu2_qTolast_so3  = Rot3::Logmap(Rot3(sensor2_qTolast_vec[i]));
        covariance_matrix += imu2_qTolast_so3 * imu1_qTolast_so3.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance_matrix.transpose() * covariance_matrix);
    Eigen::Matrix3d v = Eigen::Matrix3d::Zero();
    for (int i = 0; i < 3; i++) {
        v(i,i) = 1.0 / sqrt(eigen_solver.eigenvalues()[i]);
    }
    r_2To1 = eigen_solver.eigenvectors() * v * eigen_solver.eigenvectors().transpose() * covariance_matrix.transpose();
    cout << "r_2To1:" << r_2To1 << endl;
    cout << "test r_2To1 * r_2To1.inverse(): " << r_2To1 * r_2To1.inverse() << endl;
    cout << "test r_2To1 det: " << r_2To1.determinant() << endl;

    Eigen::Vector3d euler_angle;
    rotationMatrix_to_eulerAngles(r_2To1,euler_angle);
    cout << "eulerAngle of 2To1: " << euler_angle.transpose() * 180.0/ M_PI << endl;
    auto q = Eigen::AngleAxisd(euler_angle[0],Eigen::Vector3d::UnitX()) *
             Eigen::AngleAxisd(euler_angle[1] ,Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(euler_angle[2],Eigen::Vector3d::UnitZ());
    cout << "test r2To1 calculated by eulerAngles: " << q.toRotationMatrix() << endl;

    /// Third： calculate extrinsic of translation
    Eigen::MatrixXd A, b, x;
    A.resize(3 * motionPair_num, 3);
    b.resize( 3 * motionPair_num, 1);
    x.resize(3, 1);
    for (int i = 0; i < motionPair_num; ++i) {
        A.block<3,3>(i*3,0) = sensor1_qTolast_vec[i].matrix() - Eigen::Matrix3d::Identity();
        b.block<3,1>(i*3,0) = r_2To1 * sensor2_pTolast_vec[i] - sensor1_pTolast_vec[i];
    }
    x = (A.transpose() * A).inverse() * A.transpose() * b;
    cout << "p_imu2To1: " <<  x(0,0) << " " <<
         x(1,0) << " " <<
         x(2,0) << endl;
    p_2To1 = x;
}

/*
 * @Input: 2 sensor relative rotation seq (gtsam::Rot3)
 * @Output: rotation extrinsic: r_2To1
 */
void hand_eye_calibration_rotation(vector<gtsam::Rot3> &vec_relaRota1, vector<gtsam::Rot3> &vec_relaRota2, Eigen::Matrix3d &r_2To1)
{
    int motionPair_num = vec_relaRota1.size();
    cout << "relative rotation pair num: " << motionPair_num << std::endl;

    /// Second： calculate extrinsic of rotation
    Eigen::Matrix3d covariance_matrix = Eigen::Matrix3d::Zero();
    for (int i = 0; i < motionPair_num; ++i) {
        Eigen::Vector3d sensor1_qTolast_so3  = Rot3::Logmap(vec_relaRota1[i]);
        Eigen::Vector3d sensor2_qTolast_so3  = Rot3::Logmap(vec_relaRota2[i]);
        covariance_matrix += sensor2_qTolast_so3 * sensor1_qTolast_so3.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance_matrix.transpose() * covariance_matrix);
    Eigen::Matrix3d v = Eigen::Matrix3d::Zero();
    for (int i = 0; i < 3; i++) {
        v(i,i) = 1.0 / sqrt(eigen_solver.eigenvalues()[i]);
    }
    r_2To1 = eigen_solver.eigenvectors() * v * eigen_solver.eigenvectors().transpose() * covariance_matrix.transpose();
    cout << "r_2To1:" << r_2To1 << endl;
    //cout << "test r_2To1 * r_2To1.inverse(): " << r_2To1 * r_2To1.inverse() << endl;
    cout << "test r_2To1 det: " << r_2To1.determinant() << endl;

    Eigen::Vector3d euler_angle;
    rotationMatrix_to_eulerAngles(r_2To1,euler_angle);
    cout << "eulerAngle of 2To1: " << euler_angle.transpose() * 180.0/ M_PI << endl;
    auto q = Eigen::AngleAxisd(euler_angle[0],Eigen::Vector3d::UnitX()) *
             Eigen::AngleAxisd(euler_angle[1] ,Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(euler_angle[2],Eigen::Vector3d::UnitZ());
    //cout << "test r2To1 calculated by eulerAngles: " << q.toRotationMatrix() << endl;
}

