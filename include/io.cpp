/// Data (rosbag, traj, etc.) IO
/// Author: Weitong Wu, wwtgeomatics@gmail.com
/// Version 1.1
/// Create Time: 2022/5/24
/// Update Time: 2023/5/8

#include "io.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

bool compare_time_func (LidarData lidar_data1, LidarData lidar_data2) { return (lidar_data1.start_time_s < lidar_data2.start_time_s); }

void parseLivoxLidar(const livox_ros_driver::CustomMsgConstPtr &msg, int point_filter_num, LidarData &one_frame_lidar_data)
{
    ITPointCloud pointcloud;
    int point_num = msg->point_num;
    double timebase = msg->header.stamp.toSec();

    for (int i = 0; i < point_num; i += point_filter_num)
    {
        if (((msg->points[i].tag & 0x30) == 0x10) /// tag is in binary format and we need 00100000
            && (!std::isnan(msg->points[i].x)) && (!std::isnan(msg->points[i].y)) && (!std::isnan(msg->points[i].z)))
        {
            PointXYZIT pt;
            pt.x = msg->points[i].x;
            pt.y = msg->points[i].y;
            pt.z = msg->points[i].z;
            pt.relative_time_ms = msg->points[i].offset_time / 1000000.0; // relative time in ms
            pt.reflectivity = msg->points[i].reflectivity;
            double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if (dist > 100.0)
            {
                continue;
            }
            pointcloud.push_back(pt);
        }
    }

    one_frame_lidar_data.pointcloud = pointcloud;
    one_frame_lidar_data.start_time_s = timebase;
}

void parseHesaiLidar(const sensor_msgs::PointCloud2::ConstPtr &msg, int point_filter_num, LidarData &one_frame_lidar_data)
{
    pcl::PointCloud<hesai_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int point_num = pl_orig.points.size();
    double timebase = msg->header.stamp.toSec();

    ITPointCloud pointcloud;
    for (int i = 0; i < point_num; i += point_filter_num)
    {
        if ((!std::isnan(pl_orig.points[i].x)) && (!std::isnan(pl_orig.points[i].y)) && (!std::isnan(pl_orig.points[i].z)))
        {
            PointXYZIT pt;
            pt.x = pl_orig.points[i].x;
            pt.y = pl_orig.points[i].y;
            pt.z = pl_orig.points[i].z;
            pt.relative_time_ms = (pl_orig.points[i].time - timebase) * 1000 ;
            //std::cout << pt.relative_time_ms << " ";
            pt.reflectivity = pl_orig.points[i].intensity;
            double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if (dist > 100.0)
            {
                continue;
            }
            pointcloud.push_back(pt);
        }
    }

    one_frame_lidar_data.pointcloud = pointcloud;
    one_frame_lidar_data.start_time_s = timebase;
}

void loadLidarDataFromRosbag(std::string rosbag_path, int lidar_type, int point_filter_num,
                             std::vector<std::string> &topics,
                             float start_time, float dur_time,
                             std::vector<LidarData> &lidar_data)
{
    rosbag::Bag bag;
    bag.open(rosbag_path);
    rosbag::View view1(bag);
    ros::Time start = view1.getBeginTime() + ros::Duration(start_time);
    ros::Time end = start + ros::Duration(dur_time);
    for (auto &m : rosbag::View(bag, rosbag::TopicQuery(topics), start, end))
    {
        LidarData one_frame_lidar_data;
        switch (lidar_type)
        {
            case LIVOX:
                parseLivoxLidar(m.instantiate<livox_ros_driver::CustomMsg>(), point_filter_num, one_frame_lidar_data);
                lidar_data.push_back(one_frame_lidar_data);
                break;

            case VELO16:
                cout << "vlp-16!" << endl;
                break;

            case OUST64:
                cout << "ouster!" << endl;
                break;

            case HESAI:
                parseHesaiLidar(m.instantiate<sensor_msgs::PointCloud2>(), point_filter_num, one_frame_lidar_data);
                lidar_data.push_back(one_frame_lidar_data);
                break;

            default:
                cout << "Error LiDAR type!" << endl;
                break;
        }
    }
    bag.close();
    std::cout << "lidar data size:" << lidar_data.size() << std::endl;
}

void loadLidarDataFromFolder(std::string fold_path, int point_filter_num, std::vector<LidarData> &scan_vec)
{
    for (const auto &entry : fs::directory_iterator(fold_path))
    {
        if (entry.path().extension() == ".pcd")
        {
            LidarData scan;
            // cout << entry.path().filename() << endl;
            string filename = entry.path().filename();
            // load PCD
            PointCloudPtr pointcloud(new PointCloud());
            if (pcl::io::loadPCDFile(entry.path(),*pointcloud) != 0)
            {
                cout << "load fail: " << filename << endl;
                continue;
            }
            for (int i = 0; i < pointcloud->size(); i+= point_filter_num) {
                PointXYZIT pt;
                pt.x = pointcloud->points[i].x;
                pt.y = pointcloud->points[i].y;
                pt.z = pointcloud->points[i].z;
                scan.pointcloud.push_back(pt);
            }
            int index = filename.find(".pcd");
            // cout << filename.substr(0,index) << endl;
            double timestamp = atof(filename.substr(0, index).c_str());
            //cout << setprecision(13) << timestamp << endl;
            scan.start_time_s = timestamp; //scan_end_time
            scan_vec.push_back(scan);
        }
    }
    sort(scan_vec.begin(), scan_vec.end(), compare_time_func);
    cout << "scan_vec size: " << scan_vec.size() << endl;
}

void loadImuDataFromRosbag(std::string rosbag_path, std::vector<ImuData> &imu_data, double &dt)
{
    rosbag::Bag bag;
    bag.open(rosbag_path);
    for (auto &m : rosbag::View(bag))
    {
        if (m.isType<sensor_msgs::Imu>())
        {
            ImuData one_imu_data;
            auto imu_msg = m.instantiate<sensor_msgs::Imu>();

            one_imu_data.timestamp_s = imu_msg->header.stamp.toSec();
            one_imu_data.wm << imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z;
            one_imu_data.am << imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z;

            imu_data.push_back(one_imu_data);
        }
    }
    bag.close();
    std::cout << "imu data size:" << imu_data.size() << std::endl;

    for (int i = 0; i < imu_data.size()-1; ++i) {
        dt += imu_data[i+1].timestamp_s - imu_data[i].timestamp_s;
    }
    dt /= imu_data.size()-1;
    std::cout << "imu dt: " << dt << "s" << endl;
}

void writePoseFile(std::string path, std::vector<PoseData> &pose_data)
{
    std::ofstream ofs_pose(path);
    ofs_pose << "#Timestamp  Tx   Ty   Tz   Qx   Qy   Qz   Qw" << std::endl;
    for (int j = 0; j < pose_data.size(); ++j)
    {
        ofs_pose << std::setprecision(13) << pose_data[j].timestamp << " "
                 << pose_data[j].pose.translation().x() << " "
                 << pose_data[j].pose.translation().y() << " "
                 << pose_data[j].pose.translation().z() << " "
                 << pose_data[j].pose.rotation().quaternion()[1] << " "
                 << pose_data[j].pose.rotation().quaternion()[2] << " "
                 << pose_data[j].pose.rotation().quaternion()[3] << " "
                 << pose_data[j].pose.rotation().quaternion()[0] << std::endl;
    }
    ofs_pose.close();
}

void loadPoseFile(std::string pose_path, std::vector<PoseData> &pose_data)
{
    ifstream ifs(pose_path);
    if (!ifs) {
        cout << "pose file fail to read" << endl;
        return;
    }

    char tmp[256];
    ifs.getline(tmp, 256);///read one caption line
    while (!ifs.eof()) {
        double timestamp;
        ifs >> timestamp;

        // need this judge, there is one more newline in file end
        if (ifs.peek() == EOF)
            break;

        double tmp1[7];///format: x,y,z,qx,qy,qz,qw
        for (int i = 0; i < 7; ++i) {
            ifs >> tmp1[i];
        }

        auto r = Eigen::Quaterniond(tmp1[6],tmp1[3],tmp1[4],tmp1[5]);
        auto p = Eigen::Vector3d(tmp1[0],tmp1[1],tmp1[2]);
        PoseData pose;
        pose.timestamp = timestamp;
        pose.pose = gtsam::Pose3(gtsam::Rot3(r),p);
        pose_data.push_back(pose);
    }
    cout << "pose size: " << pose_data.size() << endl;
    ifs.close();
}