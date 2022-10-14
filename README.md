# Augbot

## Installation and Dependencies

- ROS distro: [ROS Noetic](http://wiki.ros.org/noetic/Installation)

- Ubuntu version: Ubuntu 22.04 LTS

- Gazebo version: Gazebo 11

- Alphabot2 Ros Package and Simulator (https://github.com/ssscassio/alphabot2-simulator) (criar o meu repositorio com as minhas alteracoes?)

- Hector Gazebo Plugins ()

- MQTT bridge ()

- Serial ()

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
roslaunch Augbot writeRosbag
```

## ROS architecture

![Rosgraph]
(adicionar ROSgraphs



### Description of ROS nodes

- `nome`: Subscribe to `topic name` and translate it to drive control the Alphabot2.


### Description of ROS Topics:

- `topic name`: funcao, `msg type`


## João Carvalho
