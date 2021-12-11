///////////////////////////////////////////////////////////////////////////////
// KalmanTracker.h: KalmanTracker Class Declaration

#ifndef KALMAN_H
#define KALMAN_H

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "common/frame.h"

#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <ceres/ceres.h>

#include "common/time.h"

using namespace std;
using namespace cv;

#define StateType Cube

typedef struct Cube
{
    double centerx_;
    double centery_;
    double centerz_;
    double yaw_;
    double long_;
    double width_;
    double depth_;
    double corner1_[3];
    double corner2_[3];
	pcl::PointCloud<pcl::PointXYZI>::Ptr obj_pointcloud_;
    cv::Mat frame_pic_;
	uint64_t pcds_time_;
	cv::Rect bbox_;
    Eigen::Matrix4d pose_;
};

class KfTracker
{
public:
    KfTracker(
        alignedDet detection_in
    )
    {
		init_kf(detection_in);
        m_time_since_update = 0;
		m_hits = 0;
		m_hit_streak = 0;
		m_age = 0;
		m_id = kf_count;
        estimated_vel_.setZero();
    }
    ~KfTracker()
    {
        measurement_history_.clear();
    }
    alignedDet predict();
    void update(
        alignedDet detection_in,
        const Eigen::Vector3d & vel_,
        const Eigen::Matrix3d & vel_cov_
    );
    void get_kf_vel(
        Eigen::Vector3d & vel_
    );
    void update_estimated_vel(
        const Eigen::Vector3d & vel_
    );

    static int kf_count;
	int m_time_since_update;
	int m_hits;
	int m_hit_streak;
	int m_age;
	int m_id;

    int rgb3[3];
    alignedDet detection_cur_;

    Eigen::Vector3d estimated_vel_;

private:
    cv::KalmanFilter kf_;
    cv::Mat measurement_;
	std::vector<alignedDet> measurement_history_;

	void init_kf(alignedDet detection_in);
	std::vector<float> getState(
        alignedDet detection_in
    );

    
};

class Fusion_tracker
{
private:
    vector<alignedDet> last_detection_;
    vector<alignedDet> cur_detection_;
    cv::Mat last_img_;
    cv::Mat cur_img_;
	vector<cv::Point2f> prev_pts_;
    cv::Mat prev_gray_;

    vector<KfTracker> trackers_;
    int total_frames_;
    int frame_count_;
    vector<alignedDet> predictedBoxes_;
    unsigned int trkNum_;
    unsigned int detNum_;
    vector<vector<double>> iouMatrix_;
    vector<int> HungariaAssignment_;
    set<int> unmatchedTrajectories_;
    set<int> unmatchedDetections_;
    set<int> allItems_;
    set<int> matchedItems_;
    vector<cv::Point> matchedPairs_;
    double iouThreshold_;
public:
    Fusion_tracker();
    ~Fusion_tracker();

    void tracking(
        const std::vector<alignedDet> detections_in,
        cv::Mat img_in,
        Config config_,
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
        visualization_msgs::MarkerArray & obj_vel_txt_markerarray,
        VisHandel * vis_
    );
    cv::RotatedRect alignedDet2rotaterect(alignedDet detection_in);
    double GetIOU(alignedDet bb_test, alignedDet bb_gt);
    void optical_estimator(
        cv::Mat prev_,
        cv::Mat cur_,
        const std::vector<alignedDet> & prev_detection,
        const std::vector<alignedDet> & cur_detection,
        const Config & config_,
        std::vector<Eigen::Vector2d> & obj_means,
        std::vector<Eigen::Matrix2d> & obj_covariances
    );
    void points_estimator(
        const alignedDet & prev_detection,
        const alignedDet & cur_detection,
        Eigen::Vector3d & optimal_vel,
        const Eigen::Vector2d & pix_vel,
        const Eigen::Matrix2d & pix_vel_cov,
        const Eigen::Vector3d & estimated_vel,
        Eigen::Vector3d & fused_vel,
        Eigen::Matrix3d & fused_vel_cov,
        Config config_
    );
    void vel_fusion(
        const alignedDet cur_detection,
        const alignedDet prev_detection,
        const Eigen::Vector3d points_vel,
        const Eigen::Matrix3d points_vel_cov,
        const Eigen::Vector2d pix_vel,
        const Eigen::Matrix2d pix_vel_cov,
        Eigen::Vector3d & fusion_vel,
        Eigen::Matrix3d & fusion_vel_cov,
        Config config_
    );
};

void cloud_undistortion(
    const alignedDet detection_in,
    const Eigen::Vector3d vel_,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr clouds_buffer,
	visualization_msgs::Marker & arrow_buffer,
	visualization_msgs::MarkerArray & txt_buffer
);

#endif