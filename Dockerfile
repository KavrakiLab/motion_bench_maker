FROM ros:noetic-ros-base
MAINTAINER Constantinos Chamzas chamzas@rice.edu

ENV ROS_UNDERLAY /ws/devel
ENV CATKIN_WS $(realpath $ROS_UNDERLAY/..)

# Dependencies
RUN apt-get update && \
    apt-get install -y build-essential git python3-catkin-tools libhdf5-dev libpcl-dev zip wget \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-eigen-conversions \
    ros-${ROS_DISTRO}-tf-conversions \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-fcl

RUN apt-get update && apt-get install -y --no-install-recommends \
        pkg-config \
        libglvnd-dev libglvnd-dev:i386 \
        libgl1-mesa-dev libgl1-mesa-dev:i386 \
        libegl1-mesa-dev libegl1-mesa-dev:i386 \
        libgles2-mesa-dev libgles2-mesa-dev:i386 && \

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all

COPY 10_nvidia.json /usr/share/glvnd/egl_vendor.d/10_nvidia.json


# Build dependencies
WORKDIR $ROS_UNDERLAY/..
COPY . ./src/motion_bench_maker

# Build dependencies
RUN git clone --depth 1 https://github.com/KavrakiLab/robowflex.git ./src/robowflex && \
    rm -r ./src/robowflex/robowflex_dart ./src/robowflex/robowflex_tesseract ./src/robowflex/robowflex_visualization && \
    git clone --depth 1 https://github.com/KavrakiLab/robowflex_resources ./src/robowflex_resources && \
    git clone --depth 1 --branch fix-pcl-dep  https://github.com/KavrakiLab/gl_depth_sim ./src/gl_depth_sim && \
    rm -r ./src/gl_depth_sim/roscpp_gl_depth_sim_demos && \ 
    catkin config --extend /opt/ros/$ROS_DISTRO --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build --limit-status-rate 0.001 --no-notify && \
    sed -i "s#/opt/ros/\$ROS_DISTRO/setup.bash#$ROS_UNDERLAY/setup.bash#g" /ros_entrypoint.sh
