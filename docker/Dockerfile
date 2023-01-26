FROM osrf/ros:humble-desktop

RUN apt-get update
RUN apt-get install -y xterm nano wget

# RUN apt-get install -y ros-humble-ros-ign ros-humble-ros-ign-bridge

RUN apt-get update
RUN apt-get install -y lsb-release wget gnupg

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update
RUN apt-get install -y ignition-fortress
RUN apt-get install -y ros-humble-ros-ign-bridge
RUN apt-get install -y ros-humble-teleop-twist-keyboard

# NAV2 Getting Started
#   https://navigation.ros.org/getting_started/index.html#running-the-example
RUN apt-get install -y ros-humble-navigation2
RUN apt-get install -y ros-humble-nav2-bringup
RUN apt-get install -y ros-humble-turtlebot3*

RUN useradd -m -s /bin/bash -G sudo -p '' guest
USER guest

RUN mkdir ~/urdf
COPY 01-myfirst.urdf /home/guest/urdf/

RUN mkdir ~/work
COPY visualize_lidar.sdf /home/guest/work/

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "alias myterm='xterm -fa gothic -fs 14' " >> ~/.bashrc

SHELL ["/bin/bash", "-c"]
RUN source ~/.bashrc
RUN rosdep update

# RUN mkdir ~/tmp
# COPY fortress/*.* /home/guest/tmp/

# RUN sudo chmod 777 ~/tmp/fortress.sh
# RUN ~/tmp/fortress.sh

RUN mkdir -p ~/ros2_ws/src
RUN . /opt/ros/humble/setup.sh && \
    cd ~/ros2_ws/src && \
    ros2 pkg create --build-type ament_python py_pubsub

COPY fortress/publisher_member_function.py /home/guest/ros2_ws/src/py_pubsub/py_pubsub/
COPY fortress/package.xml /home/guest/ros2_ws/src/py_pubsub/
COPY fortress/setup.py /home/guest/ros2_ws/src/py_pubsub/

RUN cd ~/ros2_ws && rosdep install -i --from-path src --rosdistro humble -y
RUN cd ~/ros2_ws && colcon build --packages-select py_pubsub


# Writing a simple publisher and subscriber (Python)
#   https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#build-and-run
# cd ros2_ws
# . install/setup.bash
# ros2 run py_pubsub talker

# RUN sudo apt-get update
# RUN sudo apt-get install -y ros-humble-navigation2
# RUN sudo apt-get install -y ros-humble-nav2-bringup
# RUN sudo apt-get install -y ros-humble-turtlebot3*


# [Solved] Problem to launch rosbot_navigation_sim.launch.py
#   https://community.husarion.com/t/solved-problem-to-launch-rosbot-navigation-sim-launch-py/1258/2
RUN echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc

RUN echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> ~/.bashrc

WORKDIR /home/guest
# ENV DISPLAY host.docker.internal:0.0
ENV DISPLAY 192.168.0.109:0.0

RUN sudo apt-get install -y pcmanfm
RUN cp -r /opt/ros/humble/share/nav2_bringup .
# ros2 launch nav2_bringup/launch/tb3_simulation_launch.py headless:=False