//frame.h
#ifndef FRAME_H
#define FRAME_H

#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <image_transport/image_transport.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "tracker/Hungarian.h"

#include "common/config.h"

#define hubLidarNum 6

struct obBBOX
{
    std::string object_type_;
    double score_;
    int bbox_x1_;
    int bbox_x2_;
    int bbox_y1_;
    int bbox_y2_;

    obBBOX(
        std::string object_type,
        double score,
        int bbox_x1,
        int bbox_x2,
        int bbox_y1,
        int bbox_y2
    ):
    object_type_(object_type),
    score_(score),
    bbox_x1_(bbox_x1),
    bbox_x2_(bbox_x2),
    bbox_y1_(bbox_y1),
    bbox_y2_(bbox_y2)
    {}
};

struct cube3d
{
    std::string object_type_;
    double confidence_;
    Eigen::Matrix<double, 8, 3> cube_vertexs_;

    cube3d(
        std::string object_type,
        double confidence,
        Eigen::Matrix<double, 8, 3> cube_vertexs
    )
    {
        object_type_ = object_type;
        confidence_ = confidence;
        cube_vertexs_ = cube_vertexs;
    }
};

typedef struct alignedDet
{
    std::string type_;
    float confidence2d_;
    float confidence3d_;
    Eigen::Matrix<double, 8, 3> vertex3d_;
    cv::Rect vertex2d_;
    pcl::PointCloud<pcl::PointXYZI> cloud_;
    cv::Mat img_;
    Eigen::Matrix4d global_pose_;
}alignedDet;

typedef std::vector<std::pair<uint64_t, pcl::PointCloud<pcl::PointXYZRGB>>> pcdWithTime;
typedef std::vector<obBBOX> frameBboxs;
typedef std::vector<cube3d> frameCubes;

typedef std::pair<uint64_t, cv::Mat> imageWithTime;
typedef std::pair<uint64_t, pcdWithTime> pcdsWithTime;
typedef std::pair<uint64_t, frameBboxs> frameBboxsWithTime;
typedef std::pair<uint64_t, frameCubes> frameCubesWithTime;
typedef std::pair<uint64_t, Eigen::Matrix4d> poseWithTime;

struct detection_object
{
    int object_num_;
    frameBboxs detection_2d_;
    frameCubes detection_3d_;
    pcl::PointCloud<pcl::PointXYZI> cloud_background_;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> cloud_objects_buffer_;
    pcl::PointCloud<pcl::PointXYZI> cloud_;

    detection_object(){}
};

class Frame
{
private:
    Config global_config_;
    cv::Mat raw_img_;
    cv::Mat label_img_;
    cv::Mat depth_img_frame_;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> pointclouds_;
    pcl::PointCloud<pcl::PointXYZRGB> frame_pointcloud_;
    frameBboxs objs_;
    frameCubes cubes_;

    // [0]->raw_img_time  [1]->label_img_time [2]->pcds_time [3~7]->pcd_time 
    uint64_t time_stamp_[9];


    detection_object frame_detections_;

public:
    Frame(
        imageWithTime& raw_img,
        imageWithTime& label_img,
        pcdsWithTime& pcd,
        frameBboxsWithTime& bbox,
        frameCubesWithTime& cube,
        Config config
    );
    ~Frame();

    cv::Mat PointCloudToDepth(
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, Config global_config);

    cv::Mat PointCloudToDepthWithintensity(
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, 
        cv::Mat & intensity_map,
        Config global_config);

    pcl::PointCloud<pcl::PointXYZRGB> depth_to_pointcloud(
        cv::Mat& depthImageIn, 
        Config global_config
    );

    pcl::PointCloud<pcl::PointXYZI>::Ptr roi_depth_to_pointcloud(
        cv::Mat& depthImageIn, 
        cv::Mat& intensity_map, 
        int x_start, 
        int y_start, 
        int x_len, 
        int y_len,
        Config global_config
    );
    
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getFramePointcloud();
    pcl::PointCloud<pcl::PointXYZI>::Ptr getBackgroundcloud();
    std::vector<pcl::PointCloud<pcl::PointXYZI>> * getObjcloud();
    pcl::PointCloud<pcl::PointXYZI>::Ptr getCloud();

    void full_detection(
        pcdsWithTime * pcd_in
    );
    void point_extraction(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_,
        const frameCubes * cubes_,
        pcl::PointCloud<pcl::PointXYZI> * background_cloud,
        std::vector<pcl::PointCloud<pcl::PointXYZI>> * obj_cloud
    );
    void detection_align(
        const std::vector<pcl::PointCloud<pcl::PointXYZI>> * obj_cloud_,
        const cv::Mat * rawimg_,
        const frameCubes * cubes_,
        const frameBboxs * objs_,
        const Eigen::Matrix4d * global_pose_,
        std::vector<alignedDet> & aligned_detections
    );

    // project the 3d detection result to 2d domain
    void detection3dProj2d(
        const cube3d * vertex3d,
        cv::Rect * output
    );
    // 3d 2d detection IoU score 
    float fusionIoU(
        const cv::Rect detection3d,
        const cv::Rect detection2d
    );
    void findHungarianAssignment(
        std::vector<std::vector<double>> iouMatrix_,
        vector<cv::Point> & results
    );
};

class VisHandel
{
private:
    ros::NodeHandle nh_;

    ros::Publisher raw_img_;
    ros::Publisher img_detection_;

    ros::Publisher raw_cloud_;
    ros::Publisher background_cloud_;
    ros::Publisher raw_obj_cloud_pub_;
    ros::Publisher undistorted_obj_cloud_pub_;

    ros::Publisher marker_3d_;
    ros::Publisher txt_marker_3d_;
    ros::Publisher obj_velocity_txt_;
    ros::Publisher obj_velocity_arrow_;

public:
    VisHandel(ros::NodeHandle nh);
    ~VisHandel();

    void txt_marker_3d_publisher(
        const std::vector<alignedDet> & detection_in
    );

    void raw_cloud_publisher(sensor_msgs::PointCloud2 cloud_in);
    void raw_obj_cloud_publisher(sensor_msgs::PointCloud2 cloud_in);
    void undistorted_obj_cloud_publisher(sensor_msgs::PointCloud2 cloud_in);
    void background_cloud_publisher(sensor_msgs::PointCloud2 cloud_in);

    void raw_img_publisher(cv::Mat img_in);
    void label_img_publisher(cv::Mat img_in);

    void obj_vel_arrow_publisher(visualization_msgs::Marker arrow_in);
    void obj_vel_txt_publisher(visualization_msgs::MarkerArray txt_in);
};

#endif //FRAME_H