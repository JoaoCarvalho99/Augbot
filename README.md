# Augbot

## Author

João Carvalho, M:ERSI student, Dep. Ciência de Computadores, Faculdade de Ciências da Universidade do Porto

## Installation and Dependencies

- ROS distro: [ROS Noetic](http://wiki.ros.org/noetic/Installation)

- Ubuntu version: Ubuntu 20.05

- Gazebo version: Gazebo 11

- Alphabot2 Ros Package and Simulator (https://github.com/JoaoCarvalho99/alphabot2-simulator)

- Hector Gazebo Plugins (http://wiki.ros.org/hector_gazebo_plugins)

- MQTT bridge (https://github.com/groove-x/mqtt_bridge)

- Serial (http://wiki.ros.org/serial)

- scipy (https://scipy.org/)

- paho (https://pypi.org/project/paho-mqtt/)

- nlohmann-json (https://pypi.org/project/paho-mqtt/)

- mock (https://pypi.org/project/mock/)

- mosquitto (https://mosquitto.org/)

# Setup your sources.list
```
sudo apt-get update && apt-get install -y lsb-release && apt-get clean all
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
# Set up your keys
```
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
```
# INSTALLATION
```
sudo apt update
sudo apt install ros-noetic-desktop-full
```
# Environment setup
```
sudo echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && source ~/.bashrc
```
# Dependencies for building packages
```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential git
sudo apt install python3-rosdep
sudo rosdep init
sudo rosdep update
```
# BUILDING 
```
sudo mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
```
# DEPENDENCIES
```
cd ~/catkin_ws/src
sudo git clone https://github.com/JoaoCarvalho99/alphabot2-simulator.git
sudo apt install ros-noetic-hector-gazebo-plugins
sudo apt install python3-pip ros-noetic-rosbridge-library mosquitto mosquitto-clients nlohmann-json3-dev 
sudo pip install paho-mqtt mock scipy
sudo apt install ros-noetic-serial
catkin_make
cd ~/catkin_ws
sudo source devel/setup.bash
sudo chmod +x src/alphabot2-simulator/**/*.py
```


Perform the full installation for the ROS Noetic that comes with Gazebo 11

<!-- ## Directory Organization -->

## MQTT configuration

- scripts/mqtt_paramls.yaml: change mqtt:connection:host: field to MQTT broker's IP

## Building

Clone this repository into the src folder inside the catkin workspace and compile it.

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/JoaoCarvalho99/Augbot.git
cd ..
catkin_make
source devel/setup.bash
chmod +x src/Augbot/scripts/*.py
chmod +x src/Augbot/scripts/AlphaBot2/python/*.py
```

## Running

### First Step
```
roscore
```

### AlphaBot2 Configuration
```
roslaunch Augbot robot.launch
sudo python3 ~/catkin_ws/src/AugBot/scripts/AlphaBot2/python/LineFollow.py
rosrun Augbot Obstacle_Detection.py
```

### Simulation Configuration
```
roslaunch Augbot spawnSimulation.launch
roslaunch Augbot simulation.launch
```

### Replay Configuration
```
rosbag play (the ROSbag file with the sensors logs to be replayed)
roslaunch Augbot replay.launch
```

### To store logs
```
roslaunch Augbot writeRosbag.launch
```

<!-- ## ROS architecture

![Rosgraph]
(adicionar ROSgraphs) -->



### Description of ROS nodes

- `UWB_Reader`: Read data from DWM1001 sensor and publishes to `/UWB`.
- `uwb_simulation`: Subscribe to `/tf` to simulate UWB ranges and publishes to `/UWB`.
- `IMU_Reader`: Read data from IMU sensor and publishes to `/IMU`.
- `deadReckoning`: Subscribe to `/IMU` and... estimates de position and publishes to `/deadReckoning`.
- `least_squares`: Subscribe to `/UWB` and estimates de position and publishes to `/leastSquares`.
- `AlphaBot2_Control`: Control the robot movement in the simulation and publishes to `/alphabot2/control`.
- `writeRosbag`: Log data from ROStopics to ROSbags.
- `mqttBridge`: Send data from ROStopics to outside of the system through MQTT.


<!-- ### Description of ROS Topics:

- `topic name`: funcao, `msg type` (todo) -->


