
# 🤖 Welcome to the Project! 

This project aims to develop a robust system for human-controlled mobile-wheeled robots to navigate unknown obstacle-filled environments. 
To navigate an obstacle-filled environment, obstacle avoidance is critical to ensure safety.
The project uses the E-Puck 2 robot, a PS4 controller and a wooden maze with wooden obstacles.

## Approach 🧠

The approach to obstacle avoidance involves sensor-based obstacle detection, followed by data collection on obstacle distance and position. A fuzzy logic controller then determines the optimal trajectory for obstacle evasion.

## Collaboration 🤝

A Mealy finite state machine continuously transitions between autonomous and user control, depending on obstacle presence. To enhance the user experience during autonomous control, the E-Puck 2's LEDs are purple, and the controller vibrates.

## For Fun Robot Videos and the Testing Process, Check out the [Videos](https://abdn.cloud.panopto.eu/Panopto/Pages/Sessions/List.aspx#folderID=%224afc57f4-35f8-4bf0-b1cb-b11700c30288%22) 📹

## OA_Control 🎛️

**OA_Control** is a Python library designed for obstacle avoidance in the ePuck 2 robotic platform. It provides a comprehensive set of functions to enable obstacle avoidance in the E-Puck 2.

## OA_Execution 🚀

**OA_Execution** is the main script executing the obstacle avoidance program on the ePuck 2 platform. It facilitates the interaction between the robot's sensors, the OA_Control library, and the E-Puck 2 robotic system.

## Features ✨

- Provides functions for obstacle detection using sensor data from the E-Puck 2.
- Implements algorithms for path planning and navigation around obstacles optimized for the E-Puck 2 platform.
- Offers customizable parameters to adjust the maximum linear and angular velocity.

## Requirements 📋

### Libraries 📚 :
- serial
- time
- struct
- pygame
- os
- tabulate
- numpy
- pandas
- openpyxl

### Hardware:
- ePuck 2 robot
- PS4 controller

### Software:
- Python environment with support for the above libraries
