# Micro ROS Demo

## Overview

The primary purpose for this repository is to organise all packages for the [Micro-ROS project](https://microros.github.io/micro-ROS/) functionalities demonstrations.
All packages contained in this repository are a part of the Micro-ROS project stack.
For more information about Micro-ROS project click [here](https://microros.github.io/micro-ROS/).

## Package clusters

The repository contains the below packages clusters:

### Simple message demonstration

#### Packages

##### Int32_publisher

The purpose of the package is to publish one of the most basic ROS 2 messages and demonstrate how Micro-ROS layers (rcl, typesupport and rmw) handle it.
For each publication, the message value increases in one unit order to see in the subscriber side the message variations.

##### Int32_subscriber

The purpose of the package is to subscribe to one of the most basic ROS 2 messages and demonstrate how Micro-ROS layers (rcl, typesupport and rmw) handle it.

#### Run demonstration

To run the demonstration, as a first step, you have to build all the required packages in a ROS 2 workspace.
To read further about how to build the ROS 2 workspace click [here](https://index.ros.org/doc/ros2/Linux-Development-Setup/).

```bash
cd ~/ros2_ws
colcon build --cmake-args -DBUILD_SHARED_LIBS=ON --packages-up-to int32_publisher int32_subscriber uros_agent
```

Configure the environment

```bash
. ~/ros2_ws/install/local_setup.bash
```

Run the Micro XRCE-DDS Agent

```bash
cd ~/ros2_ws/install/microxrcedds_agent/bin
./MicroXRCEAgent udp 8888
```

You may prefer to run the Agent in the background and discard all outputs to keep using the same terminal for the next step.

```bash
cd ~/ros2_ws/install/microxrcedds_agent/bin
./MicroXRCEAgent udp 8888 > /dev/null &
```

Run the publisher.

```bash
 ~/ros2_ws/install/int32_publisher/lib/int32_publisher/./int32_publisher
```

You may prefer to run the publisher in the background and discard all outputs to keep using the terminal for the next step.

```bash
 ~/ros2_ws/install/int32_publisher/lib/int32_publisher/./int32_publisher > /dev/null &
```

Run the subscriber.

```bash
~/ros2_ws/install/int32_subscriber/lib/int32_subscriber/./int32_subscriber
```

### String message demonstration

#### String packages

##### string_publisher

The purpose of the package is to publish a simple string ROS 2 message and demonstrate how Micro-ROS layers (rcl, typesupport and rmw) handle it.
For each publication, the message string number increases in one unit order to see in the subscriber side the message variations.

##### String_subscriber

The purpose of the package is to subscribe to a simple string ROS 2 message and demonstrate how Micro-ROS layers (rcl, typesupport and rmw) handle it.

#### Run string demonstration

To run the demonstration, as a first step, you have to build all the required packages in a ROS 2 workspace.
To read further about how to build the ROS 2 workspace click [here](https://index.ros.org/doc/ros2/Linux-Development-Setup/).

```bash
cd ~/ros2_ws
colcon build --cmake-args -DBUILD_SHARED_LIBS=ON --packages-up-to string_publisher string_subscriber uros_agent
```

Configure the environment

```bash
. ~/ros2_ws/install/local_setup.bash
```

Run the Micro XRCE-DDS Agent

```bash
cd ~/ros2_ws/install/microxrcedds_agent/bin
./MicroXRCEAgent udp 8888
```

You may prefer to run the Agent in the background and discard all outputs to keep using the same terminal for the next step.

```bash
cd ~/ros2_ws/install/microxrcedds_agent/bin
./MicroXRCEAgent udp 8888 > /dev/null &
```

Run the publisher.

```bash
 ~/ros2_ws/install/string_publisher/lib/string_publisher/./string_publisher
```

You may prefer to run the publisher in the background and discard all outputs in order still using the terminal for the next step.

```bash
 ~/ros2_ws/install/string_publisher/lib/string_publisher/./string_publisher > /dev/null &
```

Run the subscriber.

```bash
~/ros2_ws/install/string_subscriber/lib/string_subscriber/./string_subscriber
```

### Complex message demonstration

#### Complex packages

##### complex_msg

One of the purposes of the package is to demonstrate how typesupport code is generated for a complex message.
Also, the generation of a complex ROS 2 structure message is used to demonstrate how the different layers (rcl, typesupport and rmw) handle it.
The message structure contains the following types:

- All primitive data types.
- Nested message data.
- Unbonded string data.

##### Complex_msg_publisher

The purpose of the package is to publish a complex ROS 2 message and demonstrate how Micro-ROS layers (rcl, typesupport and rmw) handle it.
For each publication, the message values increases in one unit order to see in the subscriber side the message variations.

##### Complex_msg_subscriber

The purpose of the package is to subscribe to a complex ROS 2 message and demonstrate how Micro-ROS layers (rcl, typesupport and rmw) handle it.

#### Run complex demonstration

To run the demonstration, as a first step, you have to build all the required packages in a ROS 2 workspace.
To read further about how to build the ROS 2 workspace click [here](https://index.ros.org/doc/ros2/Linux-Development-Setup/).

```bash
cd ~/ros2_ws
colcon build --cmake-args -DBUILD_SHARED_LIBS=ON --packages-up-to complex_msgs complex_msg_publisher complex_msg_subscriber uros_agent
```

Configure the environment

```bash
. ~/ros2_ws/install/local_setup.bash
```

Run the Micro XRCE-DDS Agent

```bash
cd ~/ros2_ws/install/microxrcedds_agent/bin
./MicroXRCEAgent udp 8888
```

You may prefer to run the Agent in the background and discard all outputs to keep using the same terminal for the next step.

```bash
cd ~/ros2_ws/install/microxrcedds_agent/bin
./MicroXRCEAgent udp 8888 > /dev/null &
```

Run the publisher.

```bash
 ~/ros2_ws/install/complex_msg_publisher/lib/complex_msg_publisher/./complex_msg_publisher
```

You may prefer to run the Agent in the background and discard all outputs to keep using the same terminal for the next step.

```bash
 ~/ros2_ws/install/complex_msg_publisher/lib/complex_msg_publisher/./complex_msg_publisher > /dev/null &
```

Run the subscriber.

```bash
~/ros2_ws/install/complex_msg_subscriber/lib/complex_msg_subscriber/./complex_msg_subscriber
```

### Real application demonstration

This purpose of the packages is to demonstrate Micro-ROS stack can be used in a real application scenario.
In this demonstration, an altitude control system is simulated.
The primary purpose of this is to see how Micro-ROS communicates with ROS 2 nodes.

#### Real application packages

##### rad0_actuator

The mission of this node is to simulate a dummy engine power actuator.
It receives power increments and publishes the total power amount as a DDS topic.

The node is built using the Micro-ROS middleware packages (rmw_micro_xrcedds and rosidl_typesupport_microxrcedds).

It is meant to be running in a microcontroller processor, but for this demonstration, the node runs on the host PC.
The node is connected to the DDS world through a Micro XRCE-DDS Agent.

##### rad0_altitude_sensor

The mission of this node is to simulate a dummy altitude sensor.
It publishes the altitude variations as a DDS topic.

The node is built using the Micro-ROS middleware packages (rmw_micro_xrcedds and rosidl_typesupport_microxrcedds).

It is meant to be running in a microcontroller processor, but for this demonstration, the node runs on the host PC.
The node is connected to the DDS world through a Micro XRCE-DDS Agent.

##### rad0_control

The mission of this node is to read altitude values and send to the actuator engine variations.
It also publishes the status (OK, WARNING or FAILURE) as a DDS topic.
The status depends on the altitude value.

The node is built using the ROS 2 middleware packages (rmw_fastrtps and rosidl_typesupport_fastrtps).

It is meant to be running in on a regular PC, and it is directly connected to de DDS world.

##### rad0_display

The mission of this node is to simulate one LCD screen that prints the critical parameters.
It subscribes to the altitude, power and status messages available as a DDS topic.

The node is built using the Micro-ROS middleware packages (rmw_micro_xrcedds and rosidl_typesupport_microxrcedds).

It is meant to be running in a microcontroller processor, but for this demonstration, the node runs on the host PC.
The node is connected to the DDS world through a Micro XRCE-DDS Agent.

#### Run real application demonstration

Note: For this demonstration, you need at least two open terminals, one for the ROS 2 workspace and the other for the ROS 2 workspace.

##### Micro-ROS nodes

To run the demonstration, as a first step, you have to build all the required packages in a ROS 2 workspace.
To read further about how to build the ROS 2 workspace click [here](https://index.ros.org/doc/ros2/Linux-Development-Setup/).

```bash
cd ~/ros2_ws
colcon build --cmake-args -DBUILD_SHARED_LIBS=ON --packages-up-to rad0_actuator rad0_display rad0_altitude_sensor uros_agent
```

Configure the environment

```bash
. ~/ros2_ws/install/local_setup.bash
```

Run the Micro XRCE-DDS Agent

```bash
cd ~/ros2_ws/install/microxrcedds_agent/bin
./MicroXRCEAgent udp 8888
```

You may prefer to run the Agent in the background and discard all outputs to keep using the same terminal for the next steps.

```bash
cd ~/ros2_ws/install/microxrcedds_agent/bin
./MicroXRCEAgent udp 8888 > /dev/null &
```

Run the altitude_sensor node.

```bash
 ~/ros2_ws/install/rad0_altitude_sensor/lib/rad0_altitude_sensor/./rad0_altitude_sensor
```

You may prefer to run the publisher in the background and discard all outputs to keep using the terminal for the next steps.

```bash
 ~/ros2_ws/install/rad0_altitude_sensor/lib/rad0_altitude_sensor/./rad0_altitude_sensor > /dev/null &
```

Run the actuator node.

```bash
 ~/ros2_ws/install/rad0_actuator/lib/rad0_actuator/./rad0_actuator
```

You may prefer to run the publisher in the background and discard all outputs to keep using the terminal for the next steps.

```bash
 ~/ros2_ws/install/rad0_actuator/lib/rad0_actuator/./rad0_actuator > /dev/null &
```

Run the display node.

```bash
~/ros2_ws/install/rad0_display/lib/rad0_display/./rad0_display
```

##### ROS 2 nodes

To run the demonstration, you have first to build all required packages in a ROS 2 workspace.
To read further about how to build the ROS 2 workspace click [here](https://index.ros.org/doc/ros2/Linux-Development-Setup/).

```bash
cd ~/ros2_ws
colcon build --packages-up-to rad0_control
```

Configure the environment

```bash
. ~/ros2_ws/install/local_setup.bash
```

Run the display node.

```bash
ros2 run control control &
```
