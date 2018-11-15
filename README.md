# Micro ROS Demo

## Overview

The primary purpose for this repository is to organise all packages for the [Micro-ROS project](https://microros.github.io/micro-ROS/) functionalities demonstrations.
All packages contained in this repository are a part of the Micro-ROS project stack.

## Previous step

To run the all demonstrations you need to setup the ROS2 environment and build all the required packages.
Click [here](https://github.com/microROS/micro-ROS-doc) to read further about how to do this previous step.

## Package clusters

The repository contains the below packages clusters:

### Simple message demonstration

#### Packages

##### Int32_publisher

The purpose of the package is to publish one of the most basic ROS2 messages and demonstrate how Micro-ROS layers (rcl, typesupport and rmw) handle it.
For each publication, the message value increases in one unit order to see in the subscriber side the message variations.

##### Int32_subscriber

The purpose of the package is to subscribe to one of the most basic ROS2 messages and demonstrate how Micro-ROS layers (rcl, typesupport and rmw) handle it.

#### Run demonstration (Linux Debian)

Run the Micro XRCE-DDS Agent

```bash
~/ros2_WS/install/uros_agent/lib/uros_agent/./uros_agent udp 8888
```

You may prefer to run the Agent in the background and discard all outputs to keep using the same terminal for the next step.

```bash
~/ros2_WS/install/uros_agent/lib/uros_agent/./uros_agent udp 8888 > /dev/null &
```

Run the publisher.

```bash
~/ros2_WS/install/int32_publisher_c/lib/int32_publisher_c/./int32_publisher_c
```

You may prefer to run the publisher in the background and discard all outputs to keep using the terminal for the next step.

```bash
 ~/ros2_WS/install/int32_publisher_c/lib/int32_publisher_c/./int32_publisher_c > /dev/null &
```

Run the subscriber.

```bash
~/ros2_WS/install/int32_subscriber_c/lib/int32_subscriber_c/./int32_subscriber_c
```

#### Run demonstration (Windows)

Run the Micro XRCE-DDS Agent

```bash
START C:\W\install\Lib\uros_agent\uros_agent.exe udp 8888
```

You may prefer to run the Agent in the background and discard all outputs to keep using the same terminal for the next step.

```bash
START /B C:\W\install\uros_agent\lib\uros_agent\.\uros_agent udp 8888 > NUL
```

Run the publisher.

```bash
START  C:\W\install\int32_publisher_c\lib\int32_publisher_c\.\int32_publisher_c
```

You may prefer to run the publisher in the background and discard all outputs to keep using the terminal for the next step.

```bash
START /B C:\W\install\int32_publisher_c\lib\int32_publisher_c\.\int32_publisher_c > NUL
```

Run the subscriber.

```bash
START C:\W\install\int32_subscriber_c\lib\int32_subscriber_c\.\int32_subscriber_c
```


### String message demonstration

#### String packages

##### String_publisher

The purpose of the package is to publish a simple string ROS2 message and demonstrate how Micro-ROS layers (rcl, typesupport and rmw) handle it.
For each publication, the message string number increases in one unit order to see in the subscriber side the message variations.

##### String_subscriber

The purpose of the package is to subscribe to a simple string ROS2 message and demonstrate how Micro-ROS layers (rcl, typesupport and rmw) handle it.

#### Run string demonstration (Linux Debian)

Run the Micro XRCE-DDS Agent

```bash
~/ros2_WS/install/uros_agent/lib/uros_agent/./uros_agent udp 8888
```

You may prefer to run the Agent in the background and discard all outputs to keep using the same terminal for the next step.

```bash
~/ros2_WS/install/uros_agent/lib/uros_agent/./uros_agent udp 8888 > /dev/null &
```

Run the publisher.

```bash
 ~/ros2_WS/install/string_publisher_c/lib/string_publisher_c/./string_publisher_c
```

You may prefer to run the publisher in the background and discard all outputs in order still using the terminal for the next step.

```bash
 ~/ros2_WS/install/string_publisher_c/lib/string_publisher_c/./string_publisher_c > /dev/null &
```

Run the subscriber.

```bash
~/ros2_WS/install/string_subscriber_c/lib/string_subscriber_c/./string_subscriber_c
```

#### Run string demonstration (Windows)

Run the Micro XRCE-DDS Agent

```bash
START C:\W\install\uros_agent\lib\uros_agent\.\uros_agent udp 8888
```

You may prefer to run the Agent in the background and discard all outputs to keep using the same terminal for the next step.

```bash
START /B C:\W\install\uros_agent\lib\uros_agent\.\uros_agent udp 8888 > NUL
```

Run the publisher.

```bash
START C:\W\install\string_publisher_c\lib\string_publisher_c\.\string_publisher_c
```

You may prefer to run the publisher in the background and discard all outputs in order still using the terminal for the next step.

```bash
START /B C:\W\install\string_publisher_c\lib\string_publisher_c\.\string_publisher_c > NUL
```

Run the subscriber.

```bash
START /B C:\W\install\string_subscriber_c\lib\string_subscriber_c\.\string_subscriber_c
```


### Complex message demonstration

#### Complex packages

##### complex_msg

One of the purposes of the package is to demonstrate how typesupport code is generated for a complex message.
Also, the generation of a complex ROS2 structure message is used to demonstrate how the different layers (rcl, typesupport and rmw) handle it.
The message structure contains the following types:

- All primitive data types.
- Nested message data.
- Unbonded string data.

##### Complex_msg_publisher

The purpose of the package is to publish a complex ROS2 message and demonstrate how Micro-ROS layers (rcl, typesupport and rmw) handle it.
For each publication, the message values increases in one unit order to see in the subscriber side the message variations.

##### Complex_msg_subscriber

The purpose of the package is to subscribe to a complex ROS2 message and demonstrate how Micro-ROS layers (rcl, typesupport and rmw) handle it.

#### Run complex demonstration (Linux Debian)

Run the Micro XRCE-DDS Agent

```bash
~/ros2_WS/install/uros_agent/lib/uros_agent/./uros_agent udp 8888
```

You may prefer to run the Agent in the background and discard all outputs to keep using the same terminal for the next step.

```bash
~/ros2_WS/install/uros_agent/lib/uros_agent/./uros_agent udp 8888 > /dev/null &
```

Run the publisher.

```bash
 ~/ros2_WS/install/complex_msg_publisher/lib/complex_msg_publisher/./complex_msg_publisher
```

You may prefer to run the Agent in the background and discard all outputs to keep using the same terminal for the next step.

```bash
 ~/ros2_WS/install/complex_msg_publisher/lib/complex_msg_publisher/./complex_msg_publisher > /dev/null &
```

Run the subscriber.

```bash
~/ros2_WS/install/complex_msg_subscriber/lib/complex_msg_subscriber/./complex_msg_subscriber
```

#### Run complex demonstration (Windows)

Run the Micro XRCE-DDS Agent

```bash
START C:\W\install\uros_agent\lib\uros_agent\.\uros_agent udp 8888
```

You may prefer to run the Agent in the background and discard all outputs to keep using the same terminal for the next step.

```bash
START /B C:\W\install\uros_agent\lib\uros_agent\.\uros_agent udp 8888 > NUL
```

Run the publisher.

```bash
START C:\W\install\complex_msg_publisher\lib\complex_msg_publisher\.\complex_msg_publisher
```

You may prefer to run the Agent in the background and discard all outputs to keep using the same terminal for the next step.

```bash
START /B C:\W\install\complex_msg_publisher\lib\complex_msg_publisher\.\complex_msg_publisher > NUL
```

Run the subscriber.

```bash
START C:\W\install\complex_msg_subscriber\lib\complex_msg_subscriber\.\complex_msg_subscriber
```


### Real application demonstration

This purpose of the packages is to demonstrate Micro-ROS stack can be used in a real application scenario.
In this demonstration, an altitude control system is simulated.
The primary purpose of this is to demostrate how Micro-ROS communicates with ROS2 nodes.

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

The node is built using the ROS2 middleware packages (rmw_fastrtps and rosidl_typesupport_fastrtps).

It is meant to be running in on a regular PC, and it is directly connected to de DDS world.

##### rad0_display

The mission of this node is to simulate one LCD screen that prints the critical parameters.
It subscribes to the altitude, power and status messages available as a DDS topic.

The node is built using the Micro-ROS middleware packages (rmw_micro_xrcedds and rosidl_typesupport_microxrcedds).

It is meant to be running in a microcontroller processor, but for this demonstration, the node runs on the host PC.
The node is connected to the DDS world through a Micro XRCE-DDS Agent.

#### Run real application demonstration (Linux Debian)

##### Micro-ROS nodes

Run the Micro XRCE-DDS Agent

```bash
~/ros2_WS/install/uros_agent/lib/uros_agent/./uros_agent udp 8888
```

You may prefer to run the Agent in the background and discard all outputs to keep using the same terminal for the next steps.

```bash
~/ros2_WS/install/uros_agent/lib/uros_agent/./uros_agent udp 8888 > /dev/null &
```

Run the altitude_sensor node.

```bash
~/ros2_WS/install/rad0_altitude_sensor_c/lib/rad0_altitude_sensor_c/./rad0_altitude_sensor_c
```

You may prefer to run the publisher in the background and discard all outputs to keep using the terminal for the next steps.

```bash
~/ros2_WS/install/rad0_altitude_sensor_c/lib/rad0_altitude_sensor_c/./rad0_altitude_sensor_c > /dev/null &
```

Run the actuator node.

```bash
 ~/ros2_WS/install/rad0_actuator_c/lib/rad0_actuator_c/./rad0_actuator_c
```

You may prefer to run the publisher in the background and discard all outputs to keep using the terminal for the next steps.

```bash
 ~/ros2_WS/install/rad0_actuator_c/lib/rad0_actuator_c/./rad0_actuator_c > /dev/null &
```

Run the display node.

```bash
~/ros2_WS/install/rad0_display_c/lib/rad0_display_c/./rad0_display_c
```

##### ROS2 nodes

To run the demonstration, you have first to build all required packages for a [ROS2 installation](https://index.ros.org/doc/ros2/Linux-Development-Setup/#get-ros-2-0-code).
You will need to copy the rad0_control source folder inside the ROS2 source folder before build all packages. 

Run the control node.

```bash
ros2 run rad0_control_cpp rad0_control_cpp
```

#### Run real application demonstration (Windows)

##### Micro-ROS nodes

Run the Micro XRCE-DDS Agent

```bash
START C:\Ws\install\uros_agent\lib\uros_agent\.\uros_agent udp 8888
```

You may prefer to run the Agent in the background and discard all outputs to keep using the same terminal for the next steps.

```bash
START /B C:\Ws\install\uros_agent\lib\uros_agent\.\uros_agent udp 8888 > NUL
```

Run the altitude_sensor node.

```bash
START C:\Ws\install\rad0_altitude_sensor_c\lib\rad0_altitude_sensor_c\.\rad0_altitude_sensor_c
```

You may prefer to run the publisher in the background and discard all outputs to keep using the terminal for the next steps.

```bash
START /B C:\Ws\install\rad0_altitude_sensor_c\lib\rad0_altitude_sensor_c\.\rad0_altitude_sensor_c > NUL
```

Run the actuator node.

```bash
START C:\Ws\install\rad0_actuator_c\lib\rad0_actuator_c\.\rad0_actuator_c
```

You may prefer to run the publisher in the background and discard all outputs to keep using the terminal for the next steps.

```bash
START /B C:\Ws\install\rad0_actuator_c\lib\rad0_actuator_c\.\rad0_actuator_c > NUL
```

Run the display node.

```bash
START C:\Ws\install\rad0_display_c\lib\rad0_display_c\.\rad0_display_c
```

##### ROS2 nodes

To run the demonstration, you have first to build all required packages for a [ROS2 installation](https://index.ros.org/doc/ros2/Windows-Development-Setup/#getting-the-source-code).
You will need to copy the rad0_control source folder inside the ROS2 source folder before build all packages. 

Run the control node.

```bash
ros2 run rad0_control_cpp rad0_control_cpp
```