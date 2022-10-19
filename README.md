# Augbot

## Installation and Dependencies

- ROS distro: [ROS Noetic](http://wiki.ros.org/noetic/Installation)

- Ubuntu version: Ubuntu 20.05

- Gazebo version: Gazebo 11

- Alphabot2 Ros Package and Simulator (https://github.com/ssscassio/alphabot2-simulator) (criar o meu repositorio com as minhas alteracoes?)

- Hector Gazebo Plugins (http://wiki.ros.org/hector_gazebo_plugins)

- MQTT bridge (https://github.com/groove-x/mqtt_bridge)

- Serial (http://wiki.ros.org/serial)

Perform the full installation for the ROS Noetic that comes with Gazebo 11

## Directory Organization

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

### AlphaBot2 Configuration
```
roslaunch Augbot robot.launch
sudo python3 ~/catkin_ws/src/AugBot/scripts/AlphaBot2/python/LineFollow.py
rosrun Augbot Obstacle_Detection.py
```

### Simulation Configuration
```
roslaunch alphabot2_world spawn_world.launch
roslaunch alphabot2_world spawn_robot.launch
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

## ROS architecture

![Rosgraph]
(adicionar ROSgraphs



### Description of ROS nodes

- `UWB_Reader`: Read data from DWM1001 sensor and publishes to `/UWB`.
- `uwb_simulation`: Subscribe to `/tf` to simulate UWB ranges and publishes to `/UWB`.
- `IMU_Reader`: Read data from IMU sensor and publishes to `/IMU`.
- `deadReckoning`: Subscribe to `/IMU` and... estimates de position and publishes to `/deadReckoning`.
- `least_squares`: Subscribe to `/UWB` and estimates de position and publishes to `/leastSquares`.
- `AlphaBot2_Control`: Control the robot movement in the simulation and publishes to `/alphabot2/control`.
- `writeRosbag`: Log data from ROStopics to ROSbags.
- `mqttBridge`: Send data from ROStopics to outside of the system through MQTT.


### Description of ROS Topics:

- `topic name`: funcao, `msg type`


## João Carvalho
