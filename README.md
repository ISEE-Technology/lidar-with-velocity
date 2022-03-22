# Lidar with Velocity

####  A robust camera and Lidar fusion based velocity estimator to undistort the pointcloud. 

​	This repository is a barebones implementation for our paper [Lidar with Velocity : Motion Distortion Correction of Point Clouds fromOscillating Scanning Lidars](https://arxiv.org/abs/2111.09497) . It's a fusion based method to handle the oscillating scan Lidar points distortion problem, and can also provide a accurate velocity of the objects. 	<img src="./figs/result.gif" alt="result"/>

​	Here is a [Wiki](https://github.com/ISEE-Technology/lidar-with-velocity/wiki) to give a brief intro about the distortion from TOF Lidar and our proposed method. For more infomation, u can also check out the paper [arXiv](https://arxiv.org/abs/2111.09497). 

## 1. Prerequisites

**Ubuntu** and **ROS**.  Tested on Ubuntu 18.04. ROS Melodic

**Eigen 3.3.4** 

**Ceres Solver 1.14.0** 

**Opencv 3.2.0** 

## 2. Build on ROS

Clone the repository and catkin_make:

    cd ~/catkin_ws/src
    git clone https://github.com/ISEE-Technology/lidar-with-velocity
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash

## 3. Directly run

First download the [dataset](https://drive.google.com/file/d/1fYQFvZhQXK_kazsPTJ1DrmPwMEdRw6qX/view?usp=sharing) and extract it.

replace the "DATASET_PATH" in config/config.yaml with your extracted dataset path (notice the "/")

Then follow the commands blow :

    roslaunch object_undistort start.launch

there will be a Rviz window and a PCL Viewer window to show the results, press key "space" in the PCL Viewer window to process the next frame.
