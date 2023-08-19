FROM ubuntu:focal

RUN DEBIAN_FRONTEND=noninteractive apt-get update -y &&\
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        curl gnupg lsb-release &&\
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg &&\
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null &&\
    DEBIAN_FRONTEND=noninteractive apt-get update -y &&\
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
      build-essential \
      cmake \
      git \
      python3-flake8 \
      python3-flake8-docstrings \
      python3-pip \
      python3-pytest-cov \
      python3-rosdep \
      python3-setuptools \
      locales \
      wget

RUN python3 -m pip install -U \
   colcon-common-extensions \
   vcstool \
   flake8-blind-except \
   flake8-builtins \
   flake8-class-newline \
   flake8-comprehensions \
   flake8-deprecated \
   flake8-import-order \
   flake8-quotes \
   "pytest>=5.3" \
   pytest-repeat \
   pytest-rerunfailures

RUN locale-gen en_US en_US.UTF-8 &&\
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8

RUN mkdir -p /ros2_humble/src
WORKDIR /ros2_humble
RUN wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
RUN vcs import src < ros2.repos
RUN rosdep init && rosdep update
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN DEBIAN_FRONTEND=noninteractive apt-get update -y &&\
    DEBIAN_FRONTEND=noninteractive apt-get upgrade -y 
RUN rosdep install --from-paths src --ignore-src -y --rosdistro humble \
      --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
COPY messages2 src/
RUN colcon build --symlink-install

RUN rm /etc/apt/sources.list.d/ros2.list &&\
    DEBIAN_FRONTEND=noninteractive apt-get update -y &&\
    DEBIAN_FRONTEND=noninteractive apt remove -y \
        python3-catkin-pkg python3-catkin-pkg-modules
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' &&\
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - &&\
    DEBIAN_FRONTEND=noninteractive apt-get update -y &&\
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        ros-noetic-desktop-full nano
WORKDIR /        
COPY messages1 /
RUN /bin/bash -c "cd bridge_ws && source /opt/ros/noetic/setup.bash  &&\
                    catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic"

RUN mkdir -p /ros1_bridge/src
WORKDIR /ros1_bridge
RUN git clone https://github.com/ros2/ros1_bridge && cd ros1_bridge &&\
               git checkout b9f1739fd84fc877a8ec6e5c416b65aa2d782f89
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
                     . /ros2_humble/install/local_setup.bash &&\
                  colcon build --packages-select ros1_bridge --cmake-force-configure"

WORKDIR /
ENV ROS1_DISTRO=noetic
ENV ROS2_DISTRO=humble
COPY ros_entrypoint.sh /
RUN /bin/bash -c "chmod +x /ros_entrypoint.sh"
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
