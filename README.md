# h_comms

High level communications for team NUS SEDS.

## Start the simulator with 3 robots
source ~/subt_ws/install/setup.bash

ign launch -v 4 cave_circuit.ign worldName:=simple_cave_01 robotName1:=X1 robotConfig1:=X1_SENSOR_CONFIG_1 robotName2:=X2 robotConfig2:=X1_SENSOR_CONFIG_1 robotName3:=X3 robotConfig3:=X1_SENSOR_CONFIG_1

## Begin the communication channel using simple comms
source ~/subt_ws/devel/setup.bash

rosrun simple_comms simple_comms

## Use the teleop to control the vehicle
source ~/subt_ws/install/setup.bash

rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/X1/cmd_vel

## Use noroute_mesh package
In seperate terminals, run the following

rosrun noroute_mesh msg_converter.py (to run message conversion services)

rosrun noroute_mesh noroute_mesh_node X1 1 (to enable robot X1 CommsClient and communication services)

rosrun noroute_mesh noroute_mesh_node X2 2 (to enable robot X2 CommsClient and communication services))

rqt (to test services)