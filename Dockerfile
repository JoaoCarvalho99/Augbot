FROM ubuntu:20.04
MAINTAINER Joao Carvalho "joaopdemacarvalho@gmail.com"
#Setup your sources.list
RUN apt-get update && apt-get install -y lsb-release && apt-get clean all
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
#Set up your keys
RUN apt install curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
#Installation
RUN apt update
RUN apt install ros-noetic-desktop-full
#Environment setup
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && source ~/.bashrc
#Dependencies for building packages
RUN apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential git
RUN apt install python3-rosdep
RUN rosdep init
RUN rosdep update
##BUILDING 
RUN mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
RUN git clone https://github.com/JoaoCarvalho99/Augbot.git
RUN git clone https://github.com/JoaoCarvalho99/alphabot2-simulator.git
RUN git clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo.git
RUN git clone https://github.com/groove-x/mqtt_bridge.git
RUN apt install python3-pip ros-noetic-rosbridge-library mosquitto mosquitto-clients
RUN git clone https://github.com/wjwwood/serial.git
RUN cd serial
RUN make
RUN make doc
RUN make install
RUN cd ../..
RUN catkin_make
RUN source devel/setup.bash
RUN chmod +x src/Augbot/scripts/*.py
RUN chmod +x src/Augbot/scripts/AlphaBot2/python/*.py
RUN chmod +x src/alphabot2-simulator/**/*.py
