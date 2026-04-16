ARG BASE_IMAGE=osrf/ros
ARG BASE_TAG=jazzy-desktop-full

FROM ${BASE_IMAGE}:${BASE_TAG}

ENV DEBIAN_FRONTEND=noninteractive
ENV GZ_VERSION=harmonic
ARG DISTRO=jazzy

RUN apt update && apt-get update

RUN apt-get install -y ros-${DISTRO}-plotjuggler
RUN apt-get install -y ros-${DISTRO}-ros-gz
RUN apt-get install -y \
    ros-${DISTRO}-joint-state-publisher \
    ros-${DISTRO}-joint-state-publisher-gui \
    ros-${DISTRO}-gz-ros2-control \
    ros-${DISTRO}-teleop-twist-joy \
    ros-${DISTRO}-joy \
    ros-${DISTRO}-pinocchio \
    ros-${DISTRO}-ros2-control \
    ros-${DISTRO}-ros2-controllers \
    ros-${DISTRO}-xacro \
    ros-${DISTRO}-rosbag2-storage-mcap \
    ros-${DISTRO}-plotjuggler-ros \
    ros-${DISTRO}-dynamixel-sdk \
    ros-${DISTRO}-controller-manager \
    ros-${DISTRO}-effort-controllers \
    ros-${DISTRO}-moveit-kinematics \
    ros-${DISTRO}-moveit-core \
    ros-${DISTRO}-moveit-ros-planning-interface \
    python-is-python3 \
    python3-pip\
    libeigen3-dev \
    nano \
    ros-${DISTRO}-rviz2 \
    python3-rosdep \
    && apt-get clean
RUN apt-get install -y \
    ros-${DISTRO}-rmw-cyclonedds-cpp \
    && apt-get clean

RUN apt install -y libwebkit2gtk-4.1-dev \
    build-essential \
    curl \
    wget \
    file \
    libxdo-dev \
    libssl-dev \
    libayatana-appindicator3-dev \
    librsvg2-dev

RUN apt install -y cargo
RUN apt install -y npm

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs >> test.sh && sh test.sh -y

RUN apt-get install -y  curl lsb-release gnupg
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg 
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN echo " CURL DONE"

RUN apt-get update 
RUN apt-get install -y gz-harmonic
RUN echo "Ignition Installed"
ENV DEBIAN_FRONTEND=dialog

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN apt remove -y rustc


# Create the user
# RUN if id -u ${USER_UID} ; then userdel `id -un ${USER_UID}` ; fi
# RUN groupadd --gid ${USER_GID} ${USERNAME} 
# RUN useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
#     && apt-get update \
#     && apt-get install -y sudo \
#     && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
#     && chmod 0440 /etc/sudoers.d/${USERNAME}

#Change HOME environment variable
# ENV HOME=/home/${USERNAME}
# ENV USER=${USERNAME}
#USER ${USERNAME}
# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************
WORKDIR /home/miguel

#RUN sudo usermod -a -G render,avahi,video ${USERNAME}
RUN echo "source /opt/ros/${DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib/" >> ~/.bashrc

# [Optional] Set the default user. Omit if you want to keep the default as root.
CMD ["/bin/bash"]
ENTRYPOINT ["/ros_entrypoint.sh"]