FROM osrf/ros:noetic-desktop-full

##################
# libfranka build
##################

# Download and build the required franka libraries
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libpoco-dev \
    libeigen3-dev \
    libfmt-dev

RUN apt-get install -y lsb-release curl
RUN mkdir -p /etc/apt/keyrings
RUN curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | tee /etc/apt/keyrings/robotpkg.asc

RUN echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | tee /etc/apt/sources.list.d/robotpkg.list

RUN apt-get update
RUN apt-get install -y robotpkg-pinocchio

RUN git clone --recursive https://github.com/frankaemika/libfranka -b 0.15.0 # only for FR3
WORKDIR /libfranka
RUN mkdir build
WORKDIR /libfranka/build
RUN cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake -DBUILD_TESTS=OFF ..
RUN cmake --build . -j8

# Make a Debian package and install it
RUN cpack -G DEB
RUN dpkg -i libfranka*.deb

##################
# franka_ros build
##################

# Setup ROS catkin workspace
WORKDIR /catkin_ws
RUN mkdir src
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/noetic/setup.sh && catkin_init_workspace src

# Add lines to the bashrc file that source ROS
RUN echo "source /ros_entrypoint.sh" >> /root/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

RUN git clone --recursive https://github.com/frankarobotics/franka_ros -b noetic-devel src/franka_ros
RUN rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka 
ENV CMAKE_PREFIX_PATH="/opt/openrobots/lib/cmake:${CMAKE_PREFIX_PATH}"
RUN source /opt/ros/noetic/setup.sh && catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/libfranka/build
RUN source devel/setup.sh

COPY ./deps/serl_franka_controllers src/serl_franka_controllers
RUN source /opt/ros/noetic/setup.sh \
    && catkin_make -DCMAKE_BUILD_TYPE=Release

RUN sed -i 's/realtime_config: enforce/realtime_config: ignore/g' /catkin_ws/src/franka_ros/franka_control/config/franka_control_node.yaml
RUN sed -i 's/publish_rate: 30  # \[Hz\]/publish_rate: 100  # \[Hz\]/g' /catkin_ws/src/franka_ros/franka_control/config/default_controllers.yaml

RUN apt install -y python3-pip
COPY requirements.txt .
RUN python3 -m pip install -r requirements.txt

WORKDIR /root
RUN apt-get install -y wget
# manually download cuda with `wget https://developer.download.nvidia.com/compute/cuda/12.8.0/local_installers/cuda_12.8.0_570.86.10_linux.run`
RUN --mount=type=bind,source=deps,target=/root/deps cd deps && bash cuda_12.8.0_570.86.10_linux.run --no-drm --toolkit --toolkitpath=/usr/local/cuda-12.8 --silent

RUN apt-get install -y zstd
RUN python3 -m pip install "https://download.stereolabs.com/zedsdk/4.2/whl/linux_x86_64/pyzed-4.2-cp38-cp38-linux_x86_64.whl"
RUN python3 -m pip install -i https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple numpy==1.24.4
# manually download ZED_SDK_Ubuntu20_cuda12.1_v4.2.5.zstd.run to ./deps from https://www.stereolabs.com/en-hk/developers/release/4.2#82af3640d775 for Ubuntu 20
# manually run the resulting image to install zed sdk
# docker run -it -v ./deps:/mnt/deps <image_id> bash
# set +e
# adduser --disabled-password --gecos "" tmp && usermod -aG sudo tmp && passwd -d tmp
# sudo -u tmp bash /mnt/deps/ZED_SDK_Ubuntu20_cuda12.1_v4.2.5.zstd.run # accept the agreements and recommended options; the installation would cost ~20 minutes.
