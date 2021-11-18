#include <iostream>

#include "common/config.h"

Config::Config()
{
    
}

Config::~Config()
{
    
}


bool Config::readParam()
{
    std::cout << "-----------------config param-----------------" << std::endl;
    YAML::Node config = YAML::LoadFile(
        "CONFIG_YAML_PATH"
    );
    if (config["camera_intrinsic"]) 
    {
        std::vector<double> camera_intrinsic_vector = 
            config["camera_intrinsic"].as<std::vector<double>>();
        double* camera_intrinsic_array = camera_intrinsic_vector.data();
        camera_intrinsic_ = 
            Eigen::Map<Eigen::Matrix3d>(camera_intrinsic_array).transpose();
        std::cout << "\ncamera_intrinsic:\n" << camera_intrinsic_ << std::endl;
    }
    else
    { 
        return false;
    }
    if (config["camera_extrinsic"]) 
    {
        std::vector<double> camera_extrinsic_vector = 
            config["camera_extrinsic"].as<std::vector<double>>();
        double* camera_extrinsic_array = camera_extrinsic_vector.data();
        camera_extrinsic_ = 
            Eigen::Map<Eigen::Matrix4d>(camera_extrinsic_array).transpose();
        std::cout << "\ncamera_extrinsic:\n" << camera_extrinsic_ << std::endl;
    }
    else
    { 
        return false;
    }
    if (config["lidar_lidar_extrinsic"]) 
    {
        Eigen::Matrix<double ,6 ,6> lidar_lidar_extrinsic_;
        std::vector<double> lidar_lidar_extrinsic_vector = 
            config["lidar_lidar_extrinsic"].as<std::vector<double>>();
        double* lidar_lidar_extrinsic_array = lidar_lidar_extrinsic_vector.data();
        lidar_lidar_extrinsic_ = 
            Eigen::Map<Eigen::Matrix<double ,6 ,6>>(lidar_lidar_extrinsic_array).transpose();
        std::cout << "\nlidar_lidar_extrinsic:\n" << lidar_lidar_extrinsic_ << std::endl;
        
        for (size_t row_idx = 0; row_idx < 6; row_idx++)
        {
            double roll = lidar_lidar_extrinsic_(row_idx, 0);
            double pitch = lidar_lidar_extrinsic_(row_idx, 1);
            double yaw = lidar_lidar_extrinsic_(row_idx, 2);
            yaw /= 180.0/3.1415926;
            pitch /= 180.0/3.1415926;
            roll /= 180.0/3.1415926;
            Eigen::Matrix3d rotation_matrix;
            rotation_matrix = Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX());
            Eigen::Matrix4d lidar_lidar_extrinsic;
            lidar_lidar_extrinsic = Eigen::Matrix4d::Identity();
            lidar_lidar_extrinsic.topLeftCorner(3,3) = rotation_matrix;
            lidar_lidar_extrinsic.topRightCorner(3,1) = Eigen::Vector3d(
                lidar_lidar_extrinsic_(row_idx, 3),
                lidar_lidar_extrinsic_(row_idx, 4),
                lidar_lidar_extrinsic_(row_idx, 5)
            );
            lidar_lidar_ex_.push_back(lidar_lidar_extrinsic);
            std::cout << "lidar " << row_idx << " extrinsic = \n" 
                << lidar_lidar_extrinsic << "\n";
        }
        
        
    }
    else
    { 
        return false;
    }
    if (config["lidar_to_apx_extrinsic"]) 
    {
        std::vector<double> lidar_to_apx_extrinsic_vector = 
            config["lidar_to_apx_extrinsic"].as<std::vector<double>>();
        double* lidar_to_apx_extrinsic_array = lidar_to_apx_extrinsic_vector.data();
        lidar_to_apx_extrinsic_ = 
            Eigen::Map<Eigen::Matrix4d>(lidar_to_apx_extrinsic_array).transpose();
        rtk_to_lidar_extrinsic_ = lidar_to_apx_extrinsic_.inverse();
        std::cout << "\nlidar_to_apx_extrinsic:\n" 
            << lidar_to_apx_extrinsic_ << std::endl;
    }
    else
    { 
        return false;
    }
    if (config["dataset_path"]) 
    {
        dataset_path_ = config["dataset_path"].as<std::string>();
        std::cout << "\ndataset_path :\n" << dataset_path_ << std::endl;

        pose_path_ = dataset_path_ + "/global_pose.yaml";
        raw_img_path_ = dataset_path_ + "/rgb_img/";
        label_img_path_ = dataset_path_ + "/label_img/";
        pcd_path_ = dataset_path_ + "/pcd/";
        bbox_path_ = dataset_path_ + "/detection_2d/";
        cube_path_ = dataset_path_ + "/detection_3d/";
    }
    else
    { 
        return false;
    }
    if (config["camera_factor"]) 
    {
        camera_factor_ = config["camera_factor"].as<double>();
        std::cout << "\ncamera_factor :\n" << camera_factor_ << std::endl;
    }
    else
    { 
        return false;
    }
    if (config["imageRows"]) 
    {
        imageRows_ = config["imageRows"].as<double>();
        std::cout << "\nimageRows :\n" << imageRows_ << std::endl;
    }
    else
    { 
        return false;
    }
    if (config["imageCols"]) 
    {
        imageCols_ = config["imageCols"].as<double>();
        std::cout << "\nimageCols :\n" << imageCols_ << std::endl;
    }
    else
    { 
        return false;
    }
    if (config["sparse_optical_flow_param"]["maxCorners"]) 
    {
        maxCorners_ = config["sparse_optical_flow_param"]["maxCorners"].as<double>();
        std::cout << "\nmaxCorners_ :\n" << maxCorners_ << std::endl;
    }
    else
    {
        return false;
    }
    if (config["sparse_optical_flow_param"]["qualityLevel"]) 
    {
        qualityLevel_ = config["sparse_optical_flow_param"]["qualityLevel"].as<double>();
        std::cout << "\nqualityLevel_ :\n" << qualityLevel_ << std::endl;
    }
    else
    {
        return false;
    }
    if (config["sparse_optical_flow_param"]["minDistance"]) 
    {
        minDistance_ = config["sparse_optical_flow_param"]["minDistance"].as<double>();
        std::cout << "\nminDistance_ :\n" << minDistance_ << std::endl;
    }
    else
    {
        return false;
    }
    if (config["sparse_optical_flow_param"]["blockSize"]) 
    {
        blockSize_ = config["sparse_optical_flow_param"]["blockSize"].as<double>();
        std::cout << "\nblockSize_ :\n" << blockSize_ << std::endl;
    }
    else
    {
        return false;
    }
    if (config["sparse_optical_flow_param"]["Harris_k_value"]) 
    {
        Harris_k_value_ = config["sparse_optical_flow_param"]["Harris_k_value"].as<double>();
        std::cout << "\nHarris_k_value_ :\n" << Harris_k_value_ << std::endl;
    }
    else
    {
        return false;
    }
    std::cout << "-----------------config param-----------------" << std::endl;
    return true;
}