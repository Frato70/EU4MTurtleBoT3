# EU4MTurtleBoT3 - Autonomous Lunar Rover

## Project Overview
This repository contains the ROS 2 software stack and configuration files for a physical autonomous lunar rover based on the TurtleBot3 Burger platform. This work is developed for the **VP774A Design and Development Project II** course at the Högskolan i Skövde (HIS)-University of Skövde, Sweden, in WiSe 2026.

## Hardware Specifications
* **Base Platform:** TurtleBot3 Burger (Waffle-Plates)
* **Actuators:** 2x DYNAMIXEL XL430 motors
* **Embedded Controller:** OpenCR1.0
* **Compute Node:** Raspberry Pi
* **Perception:** Laser Distance Sensor LDS-02 Version, a 2D laser scanner capable of sensing 360 degrees that collects a set of data around the robot to use for SLAM (Simultaneous Localization and Mapping) and Navigation. (Effective October 2025, this LDS Sensor has been changed to the LDS-03 Version)
* **Raspberry Pi Camera Module v2:** Sony IMX219 8-megapixel sensor, connected via its 15cm ribbon cable directly into the CSI port on the Raspberry Pi. Either the camera_ros or v4l2_camera package can be used.

## Contributors
* **Contributors:** Agahan Yuldashev, Al Faiz, Luis Antonio, Ulrich Jordan, Ikram

## License
This project is released under the BSD License.
