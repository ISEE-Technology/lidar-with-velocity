//crispness_score.h
#ifndef CRISPNESS_SCORE_H
#define CRISPNESS_SCORE_H
/**************************************************************************

This header file is a implementation of the crispness metrics, which is proposed 
in the paper "Self-Calibration for a 3D Laser" by Mark Sheehan.

                                        Author: Yang Wen    Date:2021-08-20

***************************************************************************/
#include <iostream>
#include<cmath>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>

// This class is used to calculate the crispness score 
class CrispnessScore
{
  public:
    /**
     * @brief This function is used to calculate the crispness score 
     *
     * @param resolution_sigma pointcloud density resolution
     * @param normalization_factor RQE normalization factor
     * @return crispness score
    */
    static double getScore(
        const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & cloud_buffer_in,
        double resolution_sigma = 0.05,
        double normalization_factor = 300.0
    )
    {
        // create the cloud seq to be evaluated
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector;
        cloud_vector = cloud_buffer_in;
        size_t frame_count = cloud_vector.size();

        double score = 0.0;
        double sigma = resolution_sigma;
        std::cout << "resolution rate = " << sigma << std::endl;
        Eigen::Matrix3d kernel_cov = sigma * sigma * Eigen::Matrix3d::Identity();

        for (size_t frame_idx_i = 0; frame_idx_i < frame_count; frame_idx_i++)
        {
            for (size_t frame_idx_j = 0; frame_idx_j < frame_count; frame_idx_j++)
            {
                // cur cloud to be processed 
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_i = 
                    cloud_vector[frame_idx_i];
                // corresponding cloud to find the nearest point
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_j = 
                    cloud_vector[frame_idx_j];

                pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
                kdtree.setInputCloud(cloud_ptr_j);

                for (size_t k = 0; k < cloud_ptr_i->size(); k++)
                {
                    pcl::PointXYZ pt_tobe_searched = cloud_ptr_i->points[k];
                    
                    int near_points_num = 1;
                    std::vector<int> pointIdxSearched(near_points_num);
                    std::vector<float> pointDistanceSearched(near_points_num);

                    if (
                        kdtree.nearestKSearch(
                            pt_tobe_searched, 
                            near_points_num, 
                            pointIdxSearched, 
                            pointDistanceSearched) > 0
                    )
                    {
                        Eigen::Vector3d pt_i(
                            pt_tobe_searched.x, 
                            pt_tobe_searched.y, 
                            pt_tobe_searched.z);
                        Eigen::Vector3d pt_j(
                            cloud_ptr_j->points[pointIdxSearched[0]].x, 
                            cloud_ptr_j->points[pointIdxSearched[0]].y, 
                            cloud_ptr_j->points[pointIdxSearched[0]].z);



                        Eigen::Matrix3d cov_ij = 
                            (kernel_cov.inverse() + kernel_cov.inverse()).inverse();
                        Eigen::Vector3d u_ij = 
                            cov_ij * 
                            (kernel_cov.inverse() * pt_i + kernel_cov.inverse() * pt_j);

                        double cov_log_item = log(
                            (kernel_cov.inverse() + kernel_cov.inverse()).determinant() / 
                            std::pow(kernel_cov.inverse().determinant(), 2)
                        );
                        double gaussian_item = exp(
                            0.5 * (
                                (u_ij.transpose() * (cov_ij.inverse())).dot(u_ij) 
                                - (
                                    (pt_i.transpose() * (kernel_cov.inverse())).dot(pt_i) + 
                                    (pt_j.transpose() * (kernel_cov.inverse())).dot(pt_j)
                                )
                                - cov_log_item
                                - 3.0 * log(2.0 * 3.1415926)
                            )
                        );
                        double cur_score = gaussian_item / (double)cloud_ptr_i->size();
                        score += cur_score;
                    }
                    else
                    {
                        std::cout << "KdTreeFLANN FIND fail!" << std::endl;
                        exit(0);
                    }
                }
            }
            
        }

        score /= frame_count * frame_count * normalization_factor;
        return score;
    }
};

/**************************************************************************

EXAMPLE:
// create a pointcloud vector to store the sequence, then call this function
double crispness_score = CrispnessScore::getScore(cloud_buffer);

***************************************************************************/

#endif //CRISPNESS_SCORE_H