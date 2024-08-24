#docker for afli_calib
FROM osrf/ros:melodic-desktop

RUN apt-get update && apt-get install -y \
    wget \
    libgflags-dev \
    libgoogle-glog-dev \
    ros-melodic-pcl-ros \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /home/third_library

#gtsam,ceres,livox-dricer
RUN wget -O gtsam-4.0.3.tar.gz https://github.com/borglab/gtsam/archive/refs/tags/4.0.3.tar.gz \
    && tar -zxvf gtsam-4.0.3.tar.gz \
    && cd gtsam-4.0.3 \
    && mkdir build install && cd build \
    && cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON -DCMAKE_INSTALL_PREFIX=/home/third_library/gtsam-4.0.3/install .. \
    && make -j$(($(nproc) - 4)) && make install \ 
    && cd /home/third_library \
    && wget -O ceres-2.1.0.tar.gz https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.1.0.tar.gz \
    && tar -zxvf ceres-2.1.0.tar.gz \
    && cd ceres-solver-2.1.0 \
    && mkdir build install && cd build \
    && cmake -DCMAKE_INSTALL_PREFIX=/home/third_library/ceres-solver-2.1.0/install .. \
    && make -j$(($(nproc) - 4)) && make install \ 
    && cd /home/third_library \
    && git clone https://github.com/Livox-SDK/Livox-SDK.git \
    && cd Livox-SDK \
    && cd build && cmake .. && make -j$(($(nproc) - 4)) && make install \ 
    && cd /home/third_library \ 
    && mkdir -p livox_ws/src && cd livox_ws/src \ 
    && git clone https://github.com/Livox-SDK/livox_ros_driver.git && cd ../ \
    && /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make -j$(($(nproc) - 4))" \
    && echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc \
    && echo "source /home/third_library/livox_ws/devel/setup.bash" >> ~/.bashrc \
    && cp /home/third_library/gtsam-4.0.3/install/lib/libmetis-gtsam.so /usr/lib \
    && mv /usr/include/flann/ext/lz4.h /usr/include/flann/ext/lz4.h.bak \
    && mv /usr/include/flann/ext/lz4hc.h /usr/include/flann/ext/lz4.h.bak \
    && ln -s /usr/include/lz4.h /usr/include/flann/ext/lz4.h \
    && ln -s /usr/include/lz4hc.h /usr/include/flann/ext/lz4hc.h
