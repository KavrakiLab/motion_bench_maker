FROM ros:noetic-ros-base
MAINTAINER Constantinos Chamzas chamzas@rice.edu

ENV ROS_UNDERLAY /ws/devel
ENV CATKIN_WS $(realpath $ROS_UNDERLAY/..)

# Dependencies
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y build-essential git python3-catkin-tools libhdf5-dev libpcl-dev zip wget libboost-all-dev libboost-serialization-dev libeigen3-dev libtinyxml2-dev libyaml-cpp-dev libhdf5-dev \
    ros-${ROS_DISTRO}-eigen-conversions \
    ros-${ROS_DISTRO}-tf-conversions \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-fcl

# Needed to generate pointclouds 
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all

# Create catkin workspace 
WORKDIR $ROS_UNDERLAY/..
RUN catkin config --extend /opt/ros/$ROS_DISTRO --cmake-args -DCMAKE_BUILD_TYPE=Release 

# Get & Install OMPL
WORKDIR /ws/src
RUN wget -O - https://github.com/ompl/ompl/archive/1.5.0.tar.gz | tar zxf -
RUN catkin build ompl --start-with ompl --limit-status-rate 0.001 --no-notify

# Get & Install MoveIt from Source
WORKDIR /ws
RUN wstool init src && \
    wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/noetic-devel/moveit.rosinstall && \
    wstool update -t src && \
    rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} && \
    rm -r ./src/moveit/ && \  
    git clone -b noetic-devel-add-bitstar-aitstar --depth 1 https://github.com/ChamzasKonstantinos/moveit ./src/moveit 

RUN catkin build --limit-status-rate 0.001 --no-notify

# Copy the current motion benchmaker files to the workspace
RUN mkdir -p ./src/motion_bench_maker 
ADD ./ ./src/motion_bench_maker/

# Clone MotionBenchMaker and dependencies
RUN git clone --depth 1 https://github.com/KavrakiLab/robowflex.git ./src/robowflex && \
    rm -r ./src/robowflex/robowflex_dart ./src/robowflex/robowflex_tesseract ./src/robowflex/robowflex_visualization && \
    git clone --depth 1 https://github.com/KavrakiLab/robowflex_resources ./src/robowflex_resources && \
    git clone --depth 1 https://github.com/KavrakiLab/gl_depth_sim ./src/gl_depth_sim && \
    git clone --depth 1 https://github.com/TAMS-Group/bio_ik ./src/bio_ik && \
    rm -r ./src/gl_depth_sim/roscpp_gl_depth_sim_demos && \ 
    catkin build --limit-status-rate 0.001 --no-notify && \
    sed -i "s#/opt/ros/\$ROS_DISTRO/setup.bash#$ROS_UNDERLAY/setup.bash#g" /ros_entrypoint.sh
