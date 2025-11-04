

## Introduction
Pronto2 is a modular state estimation library for legged robots, designed to achieve reliable pose and velocity estimation using proprioceptive sensors such as IMUs and joint encoders, with optional integration of exteroceptive sources (such as cameras and lidar).
Originally developed to support the Robot Operating System (ROS), the framework has now been fully ported to ROS 2, with the kinematics rewritten using Pinocchio. This allows the user to load a robot model at runtime, eliminating the need for manual generation or compilation of kinematic/dynamic libraries.
Pronto2 is structured in a modular fashion, enabling users to easily configure and extend estimation pipelines. It incorporates modules for IMU processing, leg odometry, and online bias estimation via Zero-Update (ZUPT) calibration.
The library has been validated on multiple quadruped platforms, using both experiments and publicly available locomotion datasets, where reduced odometry drift and robust performance were demonstrated. Experiments conducted on the ANYmal-D platform demonstrate that our approach achieves superior performance in yaw estimation, reducing yaw error by 30\% compared to the default ANYmal state estimator.
Open-sourced and actively maintained, Pronto2 is intended for researchers, engineers, and practitioners who require a scalable, customizable, and ROS-2-native state estimator for legged robots.


## License
Pronto is released under the LGPL v2.1 license. Please see the LICENSE file attached to
this document for more information.
