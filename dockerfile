FROM ros:noetic-ros-core-focal

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1* \
    ros-noetic-pcl-ros \
    ros-noetic-cv-bridge\
    ros-noetic-libg2o \
    ros-noetic-rviz \
    ros-noetic-perception-pcl \
    ros-noetic-pcl-msgs \
    ros-noetic-vision-opencv \
    ros-noetic-husky-desktop


ENV HOME /root
WORKDIR $HOME/ros_ws/



RUN apt-get update && apt-get install -y --no-install-recommends \
    git cmake libgoogle-glog-dev libgflags-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    libpcl-dev \
    libyaml-cpp-dev \
    pcl-tools \
    python3-pip \
    python-is-python3 \
    software-properties-common \
    wget

# install dependencies
RUN add-apt-repository ppa:borglab/gtsam-release-4.0 \
    && apt-get update \
    && apt install -y libgtsam-dev libgtsam-unstable-dev \
    libpcap-dev

RUN apt-get update && apt-get install -y --no-install-recommends \
    "ros-noetic-ecl*"

RUN pip install pypcd open3d

SHELL ["/bin/bash", "-c"]

RUN cd && source /opt/ros/noetic/setup.bash \
    && wget http://ceres-solver.org/ceres-solver-2.1.0.tar.gz \
    && tar zxf ceres-solver-2.1.0.tar.gz \
    && mkdir ceres-bin \
    && cd ceres-bin \
    && cmake ../ceres-solver-2.1.0 \
    && make -j3 \
    && make install

# ADD ./src/ ./src

# quality of life packages
RUN apt-get update && apt-get install -y tmux vim

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-eigen-conversions

RUN cd && echo "source /opt/ros/noetic/setup.bash" > /root/.bashrc

RUN echo "source devel/setup.bash" >> /root/.bashrc

# enable mouse mode for tmux
RUN echo "set -g mouse on" > /root/.tmux.conf

# install ros velodyne driver
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-roslint

# install opencv
RUN apt-get update && apt-get install -y --no-install-recommends \
    libopencv-dev \
    python3-opencv \
    libgphoto2-dev \
    gphoto2

RUN pip install gphoto2

COPY ./src/ ./src
RUN apt-get update && rosdep install --from-paths src/dslr_ros src/velodyne src/usb_cam --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt install -y net-tools iputils-ping

RUN apt-get update && apt-get install -y --no-install-recommends ros-noetic-husky-simulator


