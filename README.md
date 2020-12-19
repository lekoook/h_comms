# Point-To-Point Reliable Communication
This package implements a reliable communication exchange between 2 robots.

The reliability comes from a basic acknowledgement process:

1. Sender sends message.
2. Sender waits for acknowledgement.
3. Receiver receives message.
4. Receiver replies with acknowledgement.

## Start the simulator with 3 robots
`catkin_make install`

`source ~/subt_ws/install/setup.bash`

`ign launch -v 4 cave_circuit.ign worldName:=simple_cave_01 robotName1:=X1 robotConfig1:=X1_SENSOR_CONFIG_1 robotName2:=X2 robotConfig2:=X1_SENSOR_CONFIG_1 robotName3:=X3 robotConfig3:=X1_SENSOR_CONFIG_1`

## Begin the communication node for each robot with:
`roslaunch ptp_comms node.launch name:=[NAME]`

where `[NAME]` is the name determined by subt.

Examples:

`roslaunch ptp_comms node.launch name:=X1`

`roslaunch ptp_comms node.launch name:=X2`

`roslaunch ptp_comms node.launch name:=X3`
