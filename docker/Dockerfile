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

RUN useradd -m -s /bin/bash -G sudo -p '' guest
USER guest

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "alias myterm='xterm -fa gothic -fs 14' " >> ~/.bashrc

SHELL ["/bin/bash", "-c"]
RUN source ~/.bashrc
RUN rosdep update

# RUN mkdir -p ~/ros2_ws/src
# RUN . /opt/ros/humble/setup.sh && \
#     cd ~/ros2_ws/src && \
#     ros2 pkg create --build-type ament_python py_pubsub

# RUN cd ~/ros2_ws && rosdep install -i --from-path src --rosdistro humble -y
# RUN cd ~/ros2_ws && colcon build --packages-select py_pubsub


# [Solved] Problem to launch rosbot_navigation_sim.launch.py
#   https://community.husarion.com/t/solved-problem-to-launch-rosbot-navigation-sim-launch-py/1258/2
RUN echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc

RUN echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> ~/.bashrc

WORKDIR /home/guest

RUN git clone https://github.com/teatime77/amr.git

# ENV DISPLAY host.docker.internal:0.0
ENV DISPLAY 192.168.0.109:0.0

# RUN cp -r /opt/ros/humble/share/nav2_bringup .

# docker build -t fortress .

# ros2 launch nav2_bringup/launch/tb3_simulation_launch.py headless:=False