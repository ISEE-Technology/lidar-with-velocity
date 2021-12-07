//config.h
#ifndef CONFIG_H
#define CONFIG_H

#include <thread>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Core>
#include <ros/package.h>

#include "yaml-cpp/yaml.h"

class Config
{
    friend class AssignmentDetector;
    friend class Frame;
    friend class Fusion_tracker;
    
private:

    double camera_factor_ = 256.0;
    int imageRows_ = 568;
    int imageCols_ = 1520;

    size_t integrator_threads_ = std::thread::hardware_concurrency();

    size_t max_threads_;
    std::string dataset_path_;
    std::string pose_path_, raw_img_path_, pcd_path_, label_img_path_, bbox_path_, cube_path_;

    double maxCorners_, qualityLevel_, minDistance_, blockSize_, Harris_k_value_;

    Eigen::Matrix4d lidar_to_apx_extrinsic_, rtk_to_lidar_extrinsic_;
    Eigen::Matrix3d camera_intrinsic_;
    Eigen::Matrix4d camera_extrinsic_;
    std::vector<Eigen::Matrix4d> lidar_lidar_ex_;

public:
    Config();
    ~Config();


    bool readParam();
    bool readData();
};




#endif //CONFIG_H
