# AFLI-Calib: Robust LiDAR-IMU extrinsic self-calibration based on adaptive frame length LiDAR odometry
This is the official implementation (refactor version based on the experiment code) of the following publication:

> **AFLI-Calib: Robust LiDAR-IMU extrinsic self-calibration based on adaptive frame length LiDAR odometry**<br/>
> [Weitong Wu](https://www.researchgate.net/profile/Weitong-Wu?ev=hdr_xprf), [Jianping Li](https://www.researchgate.net/profile/Jianping-Li-27), [Chi Chen](https://3s.whu.edu.cn/info/1025/1364.htm), [Bisheng Yang](https://3s.whu.edu.cn/info/1025/1415.htm), [Xianghong Zou](https://zouxianghong.github.io/), Yandi Yang, [Yuhang Xu](https://www.researchgate.net/profile/Yuhang-Xu-12), [Ruofei Zhong](https://iec.cnu.edu.cn/szdw/sssds/js2/aeeb2b52ad1e489fbb0da68b28822f2a.htm), Ruibo Chen<br/>
> *ISPRS Journal of Photogrammetry and Remote Sensing,2023,199:157-181*<br/>
> [**Paper**](https://doi.org/10.1016/j.isprsjprs.2023.04.004)

## 🔭 Introduction

<p align="center">
  <img src="pic/overview.png" alt="Network" width="80%">
</p>
<p align="justify">
<strong>Abstract:</strong> As an effective complement to common laser scanning systems, the portable laser scanning system can acquire
point clouds flexibly and quickly. Calibration between Light detection and ranging (LiDAR) sensors and inertial
measurement units (IMU) is the prerequisite for laser scanning systems to obtain high-quality point clouds.
Related methods have been proposed in the last two decades, where the global navigation satellite system (GNSS)
or high-precision calibration fields are commonly used. However, the extrinsic self-calibration of LiDAR-IMU is
challenging, due to the large distortion in single-frame point cloud caused by rapid motion and the position
errors of IMU integration which drift quickly. At the same time, the highly dynamic motion patterns of portable
devices and the changes in the scanned scene structure are not well considered in existing LiDAR odometry
methods. To take better advantage of the characteristics of non-repetitive scanning LiDAR sensor, this paper
proposes AFLI-Calib, which utilizes adaptive frame length LiDAR odometry to perform the extrinsic self-calibration
of LiDAR-IMU. <strong>Unlike LiDAR odometry methods with a fixed frame length, the LiDAR frame
length is dynamically adjusted according to the motion state of sensors and the matching stability of scenes. </strong>The
single-frame point cloud is registered to the map through a linear-based continuous-time model, eliminating the
motion distortion correction in advance. For further optimization of trajectory and extrinsic parameters, IMU raw
measurements and LiDAR observations are involved in the multi-constraint optimization, through tightly-coupled
IMU pre-integration constraints, LiDAR point-to-plane constraints, and prior constraints. The method
is fully validated using self-collected calibration data of indoor and outdoor scenes and different motion modes.
Experiments show that on the test data, the translation parameter accuracy of the method is 0.041 m, which is
56.3% higher than the state-of-the-art method. The standard deviation is significantly reduced, with translation
deviation (0.017 m, 0.024 m, 0.022 m) and rotation deviation (0.17◦, 0.25◦, 0.15◦), which verifies the robustness
of our method. The average RMSE of distances to the reference point cloud acquired by the terrestrial laser
scanning system (TLS) is 0.042 m, showing a high accuracy calibration result. Comparative experiments with the
fixed frame length LiDAR odometry method and classical “correction-then-registration” motion distortion model
further verify the superiority and effectiveness of the proposed adaptive frame length LiDAR odometry.
</p>

## 🔗 Related Works
<strong>Dataset:</strong>

[<u>WHU-Helmet Dataset</u>](https://github.com/kafeiyin00/WHU-HelmetDataset): A helmet-based multi-sensor SLAM dataset for the evaluation of real-time 3D mapping in large-scale GNSS-denied environments

## 💻 Requirements
The code has been tested on:
- Ubuntu 18.04
- ROS melodic
- GTSAM 4.0.3
- Ceres 2.1.0

## ✏️ Build & Run
### 1. How to build this project

```bash
cd ~/catkin_ws/src
git clone https://github.com/DCSI2022/AFLI_Calib.git
cd AFLI_Calib
catkin_make
```
Need solve the dependency before catkin_make, or use Docker

#### Docker (Recommended)

```
# in local
docker build -t $image_name:tag . #build custom name and tag from Dockerfile
docker run -it -v ~/catkin_ws/src/AFLI_Calib:/home/catkin_ws/src/AFLI_Calib -v /path_to_Data:/home/Data --network host -u root $image_name:tag
# in container 
cd /home/catkin_ws 
catkin_make 
source devel/setup.bash
```

### 2. RUN AFLO
Paramter description is provided in [Parameter_Descrip](./Parameter_Descrip.md). Check it!

In local
  ```
  roscore
  rviz -d ~/catkin_ws/src/AFLI_Calib/aflo_config.rviz
  ```
In container
  ```
  rosrun afli_calib afl_lidarOdometry $rosbag_path $lidar_type $lidar_topic $match_stability_threshold $motion_linearity_threshold $rosbag_start $rosbag_end
  
  ```
  we provide [test data](https://drive.google.com/file/d/1U0ycgMENDFWMKsURsyX6R29PVqw6CTrN/view?usp=drive_link), you can download it and test it with the command below!
  ```
  rosrun afli_calib afl_lidarOdometry $path_to_test_bag 1 /livox/lidar 30 0.1 0 50
  ```

### 3. RUN LiDAR-IMU extrinsic calibration
  ```
  rosrun afli_calib tight_licalib $rosbag_path $lo_path $lidar_type $lidar_topic $rosbag_start $rosbag_end %still_time
  ```

  for the calib_test.bag
  ```
  rosrun afli_calib tight_licalib $path_to_test_bag $LOG_LO_XX/pose.txt 1 /livox 0 50 10
  ```

  You can check the optimized extrinsic parameters in the LOG_calib_xx/estimated_extrinsic.txt
## Todo
- [ ] Modify parameters using yaml file

## 💡 Citation
If you find this repo helpful, please give us a star .
Please consider citing AFLI-Calib if this program benefits your project
```
@article{wu2023afli,
  title={AFLI-Calib: Robust LiDAR-IMU extrinsic self-calibration based on adaptive frame length LiDAR odometry},
  author={Wu, Weitong and Li, Jianping and Chen, Chi and Yang, Bisheng and Zou, Xianghong and Yang, Yandi and Xu, Yuhang and Zhong, Ruofei and Chen, Ruibo},
  journal={ISPRS Journal of Photogrammetry and Remote Sensing},
  volume={199},
  pages={157--181},
  year={2023},
  publisher={Elsevier}
}
```

## 🔗 Acknowledgments
We sincerely thank the excellent projects:
- [loam_livox](https://github.com/hku-mars/loam_livox) for inspiring the idea of the adaptive frame length
- [ikd-Tree](https://github.com/hku-mars/ikd-Tree) for point cloud map management;
- [GTSAM](https://github.com/borglab/gtsam) for IMU pre-integration and factor graph optimization;
- [Ceres](https://github.com/ceres-solver/ceres-solver) for auto-diff.
- [Sopuhs](https://github.com/strasdat/Sophus)
- [PCL](https://github.com/PointCloudLibrary/pcl)
- [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM)
