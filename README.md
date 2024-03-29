A ROS2 migration of the [nps_uw_multibeam_sonar package](https://github.com/Field-Robotics-Lab/nps_uw_multibeam_sonar) for the Blueview m450, Blueview p900, Oculus m1200d and Seabat f50 multibeam sonars. 

# Requirements

* ROS2 Galactic
* Gazebo 11
* NVIDIA GPU with at least 4GB memory
* NVIDIA CUDA Toolkit (installation guide [here](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html) and architecture compatibility [here](https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html))

This plugin was developed and tested using:
* CUDA 11.7
* NVIDIA driver 515.43.04

# Dependencies

* ros-galactic-image-pipeline
* ros-galactic-velodyne-simulator
* [ROS2 hydrographic messages](https://github.com/forssea-robotics/hydrographic_msgs)

# nps_uw_multibeam_sonar
Multibeam sonar Gazebo plugin with NVIDIA Cuda library 

# Wiki
All the command lines are the same as ROS1 but with the ROS2 key words.
https://github.com/Field-Robotics-Lab/dave/wiki/Multibeam-Forward-Looking-Sonar
