FROM osrf/ros:humble-desktop

RUN apt-get update
RUN apt-get install -y xterm nano
RUN apt-get update && apt-get install -y pcmanfm

# Gazebo - Docs: Binary Ubuntu Install
#       https://gazebosim.org/docs/fortress/install_ubuntu
RUN apt-get install -y lsb-release wget gnupg
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update
RUN apt-get install -y ignition-fortress

# Setting up a robot simulation (Gazebo) — ROS 2 Documentation: Humble documentation
#       https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo.html
RUN apt-get install -y ros-humble-ros-ign-bridge
RUN apt-get install -y ros-humble-teleop-twist-keyboard

# NAV2 Getting Started
#   https://navigation.ros.org/getting_started/index.html#running-the-example
RUN apt-get install -y ros-humble-navigation2
RUN apt-get install -y ros-humble-nav2-bringup
RUN apt-get install -y ros-humble-turtlebot3*

# Setting Up The URDF — Navigation 2 1.0.0 documentation
#   https://navigation.ros.org/setup_guides/urdf/setup_urdf.html#build-and-launch
RUN apt-get update
RUN apt-get install -y ros-humble-joint-state-publisher-gui
RUN apt-get install -y ros-humble-xacro

# Setting Up Odometry — Navigation 2 1.0.0 documentation
#   https://navigation.ros.org/setup_guides/odom/setup_odom.html#odometry-introduction
# RUN apt-get install -y ros-humble-gazebo-ros-pkgs
# ros-humble-gazebo-ros-pkgs is already the newest version (3.7.0-1jammy.20221207.063740).
# ros-humble-gazebo-ros-pkgs set to manually installed.

# Setting Up Odometry — Navigation 2 1.0.0 documentation
#   https://navigation.ros.org/setup_guides/odom/setup_odom.html#odometry-introduction
RUN apt-get install -y ros-humble-robot-localization

RUN useradd -m -s /bin/bash -G sudo -p '' guest
USER guest

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "alias myterm='xterm -fa gothic -fs 14' " >> ~/.bashrc

SHELL ["/bin/bash", "-c"]
RUN source ~/.bashrc
RUN rosdep update

# [Solved] Problem to launch rosbot_navigation_sim.launch.py
#   https://community.husarion.com/t/solved-problem-to-launch-rosbot-navigation-sim-launch-py/1258/2
RUN echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc

RUN echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> ~/.bashrc

WORKDIR /home/guest

# ENV DISPLAY host.docker.internal:0.0
ENV DISPLAY 192.168.0.109:0.0

