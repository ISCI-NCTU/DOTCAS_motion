# Collision Avoidance System
A collision avoidance and motion planning system provides ROS support for techman robots. TM5_700 is available for ROS Indigo.  

- Thes packages were orignially fork from [kentsai0319](https://github.com/kentsai0319), [yhtsai](https://github.com/yhtsai)

- **Maintainer: [Howard Chen](https://github.com/s880367)**

- Environment: ubuntu14.04 ROS indigo, **MoveIt 0.7.6**


## Overview

* Action interface on */follow\_joint\_trajectory* for seamless integration with __MoveIt__
* Publishes robot joint state on */joint\_states*
* Publishes TCP position on */tool\_position*
* Publishes TCP velocity on */tool\_velocity*
* Publishes TCP force on */wrench*
* Service call to set outputs on */tm\_driver/set\_io*


## Installation
First set up a catkin workspace (see [this tutorials](http://wiki.ros.org/catkin/Tutorials)).  
Then clone the repository into the src/ folder. It should look like  /path/to/your/catkin_workspace/src/techman_robot.  
Make sure to source the correct setup file according to your workspace hierarchy, then use ```catkin_make``` to compile.  
Note that this package depends on hardware_interface, and controller_manager.  
- **Dependancies**
- Hardware interface
````sudo apt-get install ros-indigo-hardware-interface````
- controller manager
````sudo apt-get install ros-indigo-controller-manager````
- industrial robot simulator
````sudo apt-get install ros-indigo-industrial-robot-simulator````



## Usage with Moveit

### test in simulation:

To bring up moveit environment in simulation mode, run:  
```
roslaunch tm700_moveit_config tm700_moveit_planning_execution.launch
```

### run with real robot:

set up networking:

1. Click on the network settings (double-arrow in the title bar) and select *Edit Connections*
2. Locate the new connection (mine was *Wired Connection 1*) and select *Edit*. Under the IPv4 tab, set:
    * address = 192.168.0.11 (or similar)
    * netmask = 255.255.255.0
3. Connect an ethernet cable and try to ping your connected robot:
    * ```ping 192.168.0.10```

To bring up moveit environment and connect to real robot, run:  
```
roslaunch tm700_moveit_config tm700_moveit_planning_execution.launch sim:=False obot_ip:=192.168.0.10
```


## Usage with Gazebo
- To bring up the simulated robot in Gazebo, run:  
```
roslaunch tm_gazebo tm700.launch
```

- To bring up the simulation in Gazebo with MoveIt-rviz
```
roslaunch tm_gazebo tm700_gazebo_moveit.launch
```
**- Snapshot :**
![](https://i.imgur.com/Qg7QBLz.png)
