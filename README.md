# Overview

The main porpuse for this repo is to organize all packages for the [Micro-ROS Poject]() funcionalities demostrations.


# Package clusters

The repository contains the belows packages clusters:


## Simple message demostration

### Packages

#### Int32_publisher

The porpuse of the package is to publicate a one of the simplest ROS2 message and demostrate how Micro-ROS down layers (rcl, typesupport and rmw) handle it. 
For each publication, the message value will be increased in one unit order to see in the subcriber side the menssage variations.


#### Int32_subscriber

The porpuse of the package is to subcribe to one of the simplest ROS2 message and demostrate how Micro-ROS down layers (rcl, typesupport and rmw) handle it. 


### Run demostration

To run the demostration you have first to build all required packages in a Micro-ROS workspace. 
To read farther about how to build the Micro-ROS workspace click [here]().

```bash
cd ~/ros2_ws
colcon build --cmake-args -DBUILD_SHARED_LIBS=ON --packages-up-to int32_publisher int32_subscriber uros_agent
```


Configure enviroment

```bash
. ~/ros2_ws/install/local_setup.bash
```


Run the Micro-xrce agent

```bash
cd ~/ros2_ws/install/micrortps_agent/bin
./MicroRTPSAgent udp 8888
```

You may prefert to run the Agent in background and discatr all output in order still using the terminal for the next step.

```bash
cd ~/ros2_ws/install/micrortps_agent/bin
./MicroRTPSAgent udp 8888 > /dev/null &
```


Run the publisher.

```bash
 ~/ros2_ws/install/int32_publisher/lib/int32_publisher/./int32_publisher
```

You may prefert to run the publisher in background and discatr all output in order still using the terminal for the next step.

```bash
 ~/ros2_ws/install/int32_publisher/lib/int32_publisher/./int32_publisher > /dev/null &
```

Run the subcriber.

```bash
~/ros2_ws/install/int32_subscriber/lib/int32_subscriber/./int32_subscriber
```

## String message demostration

### Packages

#### string_publisher

The porpuse of the package is to publicate a simple string ROS2 message and demostrate how Micro-ROS down layers (rcl, typesupport and rmw) handle it. 
For each publication, the message string number will be increased in one unit order to see in the subcriber side the menssage variations.


#### String_subscriber

The porpuse of the package is to subcribe to a simple string ROS2 message and demostrate how Micro-ROS down layers (rcl, typesupport and rmw) handle it. 


### Run demostration

To run the demostration you have first to build all required packages in a Micro-ROS workspace. 
To read farther about how to build the Micro-ROS workspace click [here]().

```bash
cd ~/ros2_ws
colcon build --cmake-args -DBUILD_SHARED_LIBS=ON --packages-up-to string_publisher string_subscriber uros_agent
```

Configure enviroment

```bash
. ~/ros2_ws/install/local_setup.bash
```


Run the Micro-xrce agent

```bash
cd ~/ros2_ws/install/micrortps_agent/bin
./MicroRTPSAgent udp 8888
```

You may prefert to run the Agent in background and discatr all output in order still using the terminal for the next step.

```bash
cd ~/ros2_ws/install/micrortps_agent/bin
./MicroRTPSAgent udp 8888 > /dev/null &
```


Run the publisher.

```bash
 ~/ros2_ws/install/string_publisher/lib/string_publisher/./string_publisher
```

You may prefert to run the publisher in background and discatr all output in order still using the terminal for the next step.

```bash
 ~/ros2_ws/install/string_publisher/lib/string_publisher/./string_publisher > /dev/null &
```

Run the subcriber.

```bash
~/ros2_ws/install/string_subscriber/lib/string_subscriber/./string_subscriber
```


## Complex message demostration

### Packages

#### complex_msg

One of the porpuses of the package is to demostrate how typesupport code is generated for a complex message. 
Also, the generation of a complex ROS2 structure message will be used to demostrate how down layers (rcl, typesupport and rmw) handle it. 
The message structure content the following types:
- All primitive data types.
- Nesting menssage data.
- Unbonded string data.


#### Complex_msg_publisher

The porpuse of the package is to publicate a complex ROS2 message and demostrate how Micro-ROS down layers (rcl, typesupport and rmw) handle it. 
For each publication, the message values will be increased in one unit order to see in the subcriber side the menssage variations.


#### Complex_msg_subscriber

The porpuse of the package is to subcribe a complex ROS2 message and demostrate how Micro-ROS down layers (rcl, typesupport and rmw) handle it. 


### Run demostration

To run the demostration you have first to build all required packages in a Micro-ROS workspace. 
To read farther about how to build the Micro-ROS workspace click [here]().

```bash
cd ~/ros2_ws
colcon build --cmake-args -DBUILD_SHARED_LIBS=ON --packages-up-to complex_msgs complex_msg_publisher complex_msg_subscriber uros_agent
```

Configure enviroment

```bash
. ~/ros2_ws/install/local_setup.bash
```


Run the Micro-xrce agent

```bash
cd ~/ros2_ws/install/micrortps_agent/bin
./MicroRTPSAgent udp 8888
```

You may prefert to run the Agent in background and discatr all output in order still using the terminal for the next step.

```bash
cd ~/ros2_ws/install/micrortps_agent/bin
./MicroRTPSAgent udp 8888 > /dev/null &
```


Run the publisher.

```bash
 ~/ros2_ws/install/complex_msg_publisher/lib/complex_msg_publisher/./complex_msg_publisher
```

You may prefert to run the publisher in background and discatr all output in order still using the terminal for the next step.

```bash
 ~/ros2_ws/install/complex_msg_publisher/lib/complex_msg_publisher/./complex_msg_publisher > /dev/null &
```

Run the subcriber.

```bash
~/ros2_ws/install/complex_msg_subscriber/lib/complex_msg_subscriber/./complex_msg_subscriber
```


## Real aplication demostration

This purpuse of the packages is to demostrate Micro-Ros stack can be used in a real aplication scenario. 
In this demostration an altitude control system will be simulated.
The main purpuse of this is to see how Micro-Ros comunicates with Ros2 nodes.

For farther information about this demostration click [here]()


### Packages

#### RAD0_actuator

The mission of this node is to simulate a dummy altitude engine power actuator. 
It will receive power increments and publicate the total power amount as a DDS topic.

The node will be built using the Micro-Ros middleware packages (rmw_micro_xrcedds and rosidl_typesupport_microxrcedds).

It is meant to be running in on a microcontroller processor but for this demostration, the node will run on the host PC.
The node will be conected to the DDS world throw a micro-xrce-agent.


#### RAD0_altitude_sensor

The mission of this node is to simulate a dummy altitude sensor. 
It will publicate the altitude variations as a DDS topic.

The node will be built using the Micro-Ros middleware packages (rmw_micro_xrcedds and rosidl_typesupport_microxrcedds).

It is meant to be running in on a microcontroller processor but for this demostration, the node will run on the host PC.
The node will be conected to the DDS world throw a micro-xrce-agent.


#### RAD0_control

The mission of this node is to read altitude values and send to the actuator engine variations.
It also will publicate the status (OK, WARNING or FAILURE) as a DDS topic.
The status will depend on the altitude value. 

The node will be built using the ROS2 middleware packages (rmw_fastrtps and rosidl_typesupport_fastrtps).

It is meant to be running in on a microcontroler processor and will be directly conected to de dds wold.

#### RAD0_display

The mission of this node is to simulate one LCD screen that will print the important parameters. 
It will subcribe to the altitude, power and status messages availables as a DDS topic.

The node will be built using the Micro-Ros middleware packages (rmw_micro_xrcedds and rosidl_typesupport_microxrcedds).

It is meant to be running in on a microcontroller processor but for this demostration, the node will run on the host PC.
The node will be conected to the DDS world throw a micro-xrce-agent.


### Run demostration

Note: For this demostraton you need at least two open termninals, one for the Micro-ROS workspace and the other for the ROS2 workspace.

#### Micro-ROS nodes

To run the demostration you have first to build all required packages in a Micro-ROS workspace. 
To read farther about how to build the Micro-ROS workspace click [here]().


```bash
cd ~/ros2_ws
colcon build --cmake-args -DBUILD_SHARED_LIBS=ON --packages-up-to rad0_actuator rad0_display rad0_altitude_sensor uros_agent
```

Configure enviroment

```bash
. ~/ros2_ws/install/local_setup.bash
```


Run the Micro-xrce agent

```bash
cd ~/ros2_ws/install/micrortps_agent/bin
./MicroRTPSAgent udp 8888
```

You may prefert to run the Agent in background and discatr all output in order still using the terminal for the next step.

```bash
cd ~/ros2_ws/install/micrortps_agent/bin
./MicroRTPSAgent udp 8888 > /dev/null &
```

Run the altitude_sensor node.

```bash
 ~/ros2_ws/install/rad0_altitude_sensor/lib/rad0_altitude_sensor/./rad0_altitude_sensor
```

You may prefert to run the publisher in background and discatr all output in order still using the terminal for the next step.

```bash
 ~/ros2_ws/install/rad0_altitude_sensor/lib/rad0_altitude_sensor/./rad0_altitude_sensor > /dev/null &
```

Run the actitude node.

```bash
 ~/ros2_ws/install/rad0_actuator/lib/rad0_actuator/./rad0_actuator
```

You may prefert to run the publisher in background and discatr all output in order still using the terminal for the next step.

```bash
 ~/ros2_ws/install/rad0_actuator/lib/rad0_actuator/./rad0_actuator > /dev/null &
```

Run the display node.

```bash
~/ros2_ws/install/rad0_display/lib/rad0_display/./rad0_display
```

#### ROS2 nodes

To run the demostration you have first to build all required packages in a ROS2 workspace. 
To read farther about how to build the ROS2 workspace click [here]().


```bash
cd ~/ros2_ws
colcon build --packages-up-to rad0_control
```

Configure enviroment

```bash
. ~/ros2_ws/install/local_setup.bash
```

Run the display node.

```bash
ros2 run control control &
```
