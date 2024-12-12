FROM nvidia/cuda:11.8.0-base-ubuntu22.04

# Prevent interactive prompts and set timezone to Europe/Paris
ENV DEBIAN_FRONTEND=noninteractive TZ=Europe/Paris

# Install base dependencies and configure locales
RUN apt update && apt upgrade -y && \
    apt install -y sudo wget curl git terminator grep nano mesa-utils locales tzdata gnome-terminal dbus-x11 libcanberra-gtk-module libcanberra-gtk3-module && \
    ln -fs /usr/share/zoneinfo/Europe/Paris /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8

# Add ROS 2 repository and key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update


# Clone and build PX4 Autopilot
RUN cd ~ && \
    git config --global http.postBuffer 524288000 && \
    git clone --recursive https://github.com/PX4/PX4-Autopilot.git && \
    cd PX4-Autopilot && \
    git checkout v1.15.2 && \
    git submodule sync --recursive && \
    git submodule update --init --recursive && \
    bash ./Tools/setup/ubuntu.sh && \
    make px4_sitl -j$(nproc)

# Install ROS 2, PX4 dependencies, and Gazebo
RUN apt install -y ros-humble-desktop ros-dev-tools python3-colcon-common-extensions \
    ros-humble-eigen3-cmake-module ros-humble-gazebo-msgs ros-humble-gazebo-ros-pkgs \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x \
    gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio \
    gazebo libgazebo11 libgazebo-dev && \
    apt-get clean && rm -rf /var/lib/apt/lists/*


# Setup Gazebo classic for x500
RUN cd ~ && \
    git clone https://github.com/ReconDronePilot/gazebo-classic_x500.git && \
    cd gazebo-classic_x500 && \
    chmod +x add_gazebo-classic_x500.sh && \
    ./add_gazebo-classic_x500.sh /root

# Install Micro-XRCE-DDS-Agent
RUN cd ~ && \
    git clone --recursive https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && make install && ldconfig /usr/local/lib && \
    cd && rm -rf Micro-XRCE-DDS-Agent

# Setup my workspace
RUN mkdir -p ~/ros_ws/src &&\
    cd ~/ros_ws/src && \
    git clone https://github.com/PX4/px4_msgs.git && \
    rm -rf /root/ros_ws/src/px4_msgs/srv/*  && \
    rm -rf  /root/ros_ws/src/px4_msgs/msg/* && \
    cp -r ~/PX4-Autopilot/srv/* /root/ros_ws/src/px4_msgs/srv/ && \
    cp -r ~/PX4-Autopilot/msg/* /root/ros_ws/src/px4_msgs/msg/ && \
    cd .. && \
    echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo 'cd ~/ros_ws' >> ~/.bashrc

# Set default command
CMD ["/usr/bin/terminator"]

