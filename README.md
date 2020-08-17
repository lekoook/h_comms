# h_comms
High level communications for team NUS SEDS.

## Install these prerequisites for ROS message conversion
sudo apt-get install python3-pip python3-yaml
sudo pip3 install rospkg catkin_pkg

## Start the simulator with 3 robots
source ~/subt_ws/install/setup.bash

ign launch -v 4 cave_circuit.ign worldName:=simple_cave_01 robotName1:=X1 robotConfig1:=X1_SENSOR_CONFIG_1 robotName2:=X2 robotConfig2:=X1_SENSOR_CONFIG_1 robotName3:=X3 robotConfig3:=X1_SENSOR_CONFIG_1

## Using noroute_mesh package
Make sure the workspace has been sourced.

First you need to launch the ROS messages converter ROS service:
```
roslaunch noroute_mesh converter_service.launch
```

For each robot in simulation, you need to run a communication node for it:
```
roslaunch noroute_mesh comms_node.launch name:={robotName} id:={robotId} 
```
`{robotName}` is to be replaced (including '{ }') with the name of the robot. It should not contain spaces. The names should corresponds to the name used by subt simulation.

`{robotId}` is to be replaced (including '{ }') with an identifier number for the robot. It should not contain spaces or special characters(just digits), any number within the range [0, 4294967295]. This identifier must be unique and cannot be one that already exists.

Example:
```
roslaunch noroute_mesh comms_node.launch name:=X1 id:=1 
```
