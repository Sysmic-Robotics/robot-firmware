# Sysmic Robotics Firmware

Repository of **Sysmic Robotics** hardware firmware STM32f7. Here you will find code related to robots functionality and libraries implemented for it.

# FreeRTOS
The robot's behavior is governed by its FreeRTOS operating system.

This paradigm involves establishing processes (in this case, functions) with certain priorities, and then the operating system manages when to execute these processes.

## driveTask
This task is responsible for controlling the robot's movement. It uses sensor data and received commands to adjust the speed and direction of the robot's motors.

## radioTask
This task handles the robot's wireless communication. It is responsible for sending and receiving data through a radio frequency module, allowing interaction with other devices and base stations.

## kickTask
This task controls the robot's kicking mechanism. It manages the activation system of the kicking mechanism, waiting for the radio to indicate when to kick.
