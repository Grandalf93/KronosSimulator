# -----------------------------------------------------------------------------
# Charlienet (carlos.alvarez@kiwibot.com)
# JohnBetaCode (john@kiwibot.com)
# dadaroce (davidson@kiwibot.com)
# -----------------------------------------------------------------------------

# This base image is based on nvidia/cuda:11.1.1-cudnn8-devel-ubuntu20.04 and 
# compiled opencv 4.4.0 with CUDA, tensorRT7.2 and tkdNN. Refer to .devcontainer/base/Dockerfile
FROM gokiwibot/kronos-dev:gazebo

# Or your actual UID, GID on Linux if not the default 1000
ARG USERNAME=ada
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Avoid warnings by switching to noninteractive
ENV DEBIAN_FRONTEND=noninteractive

# For wifi configuration and settings
ENV DBUS_SYSTEM_BUS_ADDRESS=unix:path=/host/run/dbus/system_bus_socket


# Nvidia key URL
# From Tensort installation instructions
ENV CUDA_KEY=/var/nv-tensorrt-repo-cuda11.1-trt7.2.2.3-ga-20201211/7fa2af80.pub
ENV NVIDIA_KEY_URL=https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-keyring_1.0-1_all.deb
RUN wget -q $NVIDIA_KEY_URL \
    && apt-key del 7fa2af80 \     
    && dpkg -i cuda-keyring_1.0-1_all.deb \
    && rm cuda-keyring_1.0-1_all.deb \
    && apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/3bf863cc.pub \
    && apt-key add $CUDA_KEY \
    && apt-get update \
    && apt autoremove -y && apt clean -y \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    apt-transport-https \
    ca-certificates \
    dialog \
    gnupg-agent \
    libasio-dev \
    libwebsocketpp-dev \
    python3 \
    python3-argcomplete \
    python3-tk \
    software-properties-common \
    nano \
    htop \
    wireless-tools \
    # ------------------------------
    # Create a non-root user to use if preferred - see https://aka.ms/vscode-remote/containers/non-root-user.
    && apt-get install -y nodejs \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # ------------------------------
    # [Optional] Add sudo support for non-root user
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    # ------------------------------
    # WebCam Permission
    && usermod -a -G video $USERNAME \
    # ------------------------------
    # Clean up
    && apt-get autoremove && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    # ------------------------------
    # v4l2loopback
    v4l2loopback-utils \
    # ------------------------------
    # For wifi configuration and settings
    network-manager \
    # ------------------------------
    # To modify ports 
    busybox \
    # ------------------------------
    # Install clang formater
    clang-format \
    clang \
    # ------------------------------
    # Tools for video and audio codecs converter and other related functions
    ffmpeg \ 
    mpg321 \
    # ------------------------------
    # Clean up
    && apt-get autoremove && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# Installing ros2
# https://index.ros.org/doc/ros2/Installation/Crystal/Linux-Install-Debians/
ENV ROS_DISTRO=galactic

# Editors and so on
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \ 
    && apt update && apt install --no-install-recommends -y \ 
    # Base Installation (Bare Bones): Communication libraries, message packages.
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-rclcpp-cascade-lifecycle \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-launch-testing-ament-cmake \
    ros-${ROS_DISTRO}-diagnostic-updater \ 
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-test-msgs \
    ros-${ROS_DISTRO}-bond-core \
    # ros-${ROS_DISTRO}-gazebo-ros \
    # ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    # ros-${ROS_DISTRO}-gazebo-msgs \
    # ros-${ROS_DISTRO}-gazebo-plugins \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-message-filters \
    ros-${ROS_DISTRO}-image-transport \ 
    ros-${ROS_DISTRO}-image-geometry \ 
    ros-${ROS_DISTRO}-rqt* \
    ros-${ROS_DISTRO}-image-geometry \
    # ------------------------------
    # Nav2 and slam toolbox required dependencies
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-behaviortree-cpp-v3 \
    ros-${ROS_DISTRO}-angles \
    ros-${ROS_DISTRO}-ompl \ 
    # required for compiling STVL plugin
    ros-${ROS_DISTRO}-pcl-conversions \
    libopenvdb-dev \
    python3-colcon-common-extensions \
    # ------------------------------
    && apt autoremove && apt clean -y \
    && rm -rf /var/lib/apt/lists/* \
    # ------------------------------
    # Rosedep
    && pip3 install rosdep \ 
    && rosdep init \
    && rosdep update \
    # ROS2 source setup
    && ROS_SOURCE="source /opt/ros/${ROS_DISTRO}/setup.sh" \
    && pip3 install -U argcomplete \
    && echo $ROS_SOURCE >> "/home/${USERNAME}/.bashrc"

WORKDIR /home/gazebo_ws/src
RUN git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git --branch galactic \
    && cd ../ \
    # Compile code
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install \
    --cmake-clean-cache \
    --cmake-args -DBUILD_TESTING=OFF \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && echo "source /home/gazebo_ws/install/setup.bash" >> "/home/${USERNAME}/.bashrc"

# Install ROS2 gazebo dependencies
RUN apt update && apt-get install --no-install-recommends -y \
    libglvnd0 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    libblkid-dev \
    e2fslibs-dev \
    libboost-all-dev \
    libaudit-dev \
    # for geometry messages in python
    python3-pykdl \
    # ------------------------------
    && apt autoremove && apt clean -y \ 
    && rm -rf /var/lib/apt/lists/*

RUN \
    mkdir -p /home/${USERNAME}/.ignition/fuel/ \
    && echo "servers:\n  -\n    name: osrf\n    url: https://api.ignitionrobotics.org" >> /home/${USERNAME}/.ignition/fuel/config.yaml \
    && chown ${USERNAME} /home/${USERNAME}/.ignition \
    && GAZEBO_SOURCE="source /usr/share/gazebo/setup.sh" \
    && echo $GAZEBO_SOURCE >> "/home/${USERNAME}/.bashrc" \
    && chown ${USERNAME} /home/${USERNAME}/.ignition

# ------------------------------------------------------------------------------
# ROS2 CONFIGS - ROS2 CONFIGS - ROS2 CONFIGS - ROS2 CONFIGS - ROS2 CONFIGS - ROS2

RUN pip install gsutil

# Logging variables
# https://index.ros.org/doc/ros2/Tutorials/Logging-and-logger-configuration/
# # If you would like more or less verbose formatting
ENV RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}][{time}][{name}]: {message}"
# If you would like to force enabling or disabling it
# the output is colorized when it’s targeting a terminal. 
ENV RCUTILS_COLORIZED_OUTPUT=1                                   
# By default, info and debug log calls aren’t line buffered. 
# You can force it using 
# ENV RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED=1   # deprecated
ENV RCUTILS_LOGGING_USE_STDOUT=1   
ENV RCUTILS_LOGGING_BUFFERED_STREAM=1

# -----------------------------------------------------------------------------
# disable the creation of core dump files on a UNIX or Linux environment
# https://dev.intelrealsense.com/docs/troubleshooting
# https://www.ibm.com/support/pages/how-disable-creation-core-dump-files-unix-or-linux
ENV ulimit -c 0

# -----------------------------------------------------------------------------
# Switch back to dialog for any ad-hoc use of apt-get
ENV DEBIAN_FRONTEND=
CMD ["/bin/bash"]

# -----------------------------------------------------------------------------
