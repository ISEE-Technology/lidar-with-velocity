# Lidar with Velocity

####  A robust camera and Lidar fusion based velocity estimator to undistort the pointcloud. 

**related paper:** Lidar with Velocity : Motion Distortion Correction of Point Clouds fromOscillating Scanning Lidars [arXiv]()

## 1. Prerequisites

1.1 **Ubuntu** and **ROS**.  Tested on Ubuntu 18.04. ROS Melodic

1.2 **Eigen**

1.3 **Ceres Solver** 

1.4 **Opencv** 

## 2. Build on ROS

Clone the repository and catkin_make:

    cd ~/catkin_ws/src
    git clone https://github.com/ISEE-Technology/lidar-with-velocity
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash



## 3. Directly run

First download our dataset [data](https://drive.google.com/drive/folders/1JEwnVVO84peunFiCXSc-T5QyK0gD3kAt?usp=sharing) and extract in /catkin_ws/ path.

replace the "DATASET_PATH" in config/config.yaml with your extracted dataset path, example: (notice the "/")

    dataset_path: YOUR_CATKIN_WS_PATH/catkin_ws/data/

replace the "CONFIG_YAML_PATH" with your config.yaml file path, example:

    "YOUR_CATKIN_WS_PATH/catkin_ws/src/lidar-with-velocity/config.yaml"

Then follow the commands blow :

    roscore
    rviz -d src/lidar-with-velocity/rviz-cfg/vis.rviz
    rosrun lidar-with-velocity main_ros

there will be a Rviz window and a PCL Viewer window to show the results, press key "space" to process the next frame.
