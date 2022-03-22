#include "common/frame.h"

Frame::Frame(
    imageWithTime& raw_img,
    imageWithTime& label_img,
    pcdsWithTime& pcd,
    frameBboxsWithTime& bbox,
    frameCubesWithTime& cube,
    Config config
)
{
    // if (
    //     bbox.first != label_img.first || 
    //     bbox.first != raw_img.first || 
    //     label_img.first != raw_img.first  
    // )
    // {
    //     throw("Construct ERROR! input imgs/labels/bboxs don't match!");
    // }

    this->global_config_ = config;

    this->raw_img_ = raw_img.second;
    this->time_stamp_[0] = raw_img.first;

    this->label_img_ = label_img.second;
    this->time_stamp_[1] = label_img.first;

    this->time_stamp_[2] = pcd.first;

    this->objs_ = bbox.second;

    this->cubes_ = cube.second;

    for (size_t pcd_idx = 0; pcd_idx < hubLidarNum; pcd_idx++)
    {
        this->time_stamp_[pcd_idx+3] = pcd.second[pcd_idx].first;
        this->pointclouds_.push_back(pcd.second[pcd_idx].second);
        frame_pointcloud_ = pcd.second[pcd_idx].second + frame_pointcloud_;
    }
}


bool Frame::verboseFrame()
{
    std::cout << "-------------------------------" << std::endl;;
    std::cout << "raw  img  time : " 
        << std::to_string(this->time_stamp_[0]).insert(10,"_") << std::endl;
    std::cout << "label img time : " 
        << std::to_string(this->time_stamp_[1]).insert(10,"_") << std::endl;
    std::cout << "pcds      time : " 
        << std::to_string(this->time_stamp_[2]).insert(10,"_") 
        << ",  points number = " << this->frame_pointcloud_.size()
        << std::endl;
    std::cout << "detected \"" << this->objs_.size() << "\" objs" << std::endl; 
    for (size_t pcd_idx = 0; pcd_idx < this->pointclouds_.size(); pcd_idx++)
    {
        std::cout << "lidar " << pcd_idx+1 << " time : " 
            <<  std::to_string(this->time_stamp_[pcd_idx+3]).insert(10,"_")
            << std::endl;
    }
    std::cout << "\n";
}

pcl::PointCloud<pcl::PointXYZRGB> Frame::depth_to_pointcloud(
    cv::Mat& depthImageIn, Config global_config
)
{
    double camera_fx = global_config.camera_intrinsic_(0,0);
    double camera_fy = global_config.camera_intrinsic_(1,1);
    double camera_cx = global_config.camera_intrinsic_(0,2);
    double camera_cy = global_config.camera_intrinsic_(1,2);
    int n,m;
    pcl::PointCloud<pcl::PointXYZRGB> cloudOut;
    for (int m = 0; m < depthImageIn.rows; m++)
    {
        for (int n=0; n < depthImageIn.cols; n++)
        {
            float d = depthImageIn.ptr<uint16_t>(m)[n];
            pcl::PointXYZRGB p;
            p.x = d / global_config.camera_factor_;
            if (p.x <= 1.0 || p.x > 260) 
            {
                continue;
            }
            p.y = -  (((double)n - camera_cx) * p.x / camera_fx);
            p.z = -  (((double)m - camera_cy) * p.x / camera_fy);
            p.r = 0;
            p.g = 255;
            p.b = 0;
            cloudOut.points.push_back(p);
        }
    }
    return cloudOut;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Frame::roi_depth_to_pointcloud
    (
    cv::Mat& depthImageIn, 
    cv::Mat& intensity_map, 
    int x_start, 
    int y_start, 
    int x_len, 
    int y_len, 
    Config global_config)
{
    double camera_fx = global_config.camera_intrinsic_(0,0);
    double camera_fy = global_config.camera_intrinsic_(1,1);
    double camera_cx = global_config.camera_intrinsic_(0,2);
    double camera_cy = global_config.camera_intrinsic_(1,2);
    int n,m;
    int r,g,b;
    // r = (rand() % 155) + 100;
    // g = (rand() % 155) + 100;
    // b = (rand() % 155) + 100;
    pcl::PointCloud<pcl::PointXYZI> cloudOut;
    for (int m = y_start; m < y_start + y_len; m++)
    {
        for (int n=0; n < x_start + x_len; n++)
        {
            float d = depthImageIn.ptr<ushort>(m)[n];
            if (d == 0) continue;
            pcl::PointXYZI p;
            p.x = d / global_config.camera_factor_;
            // if (p.x <= 0.5 || p.x > 260) 
            // {
            //     continue;
            // }
            p.y = -  (((double)n - camera_cx) * p.x / camera_fx);
            p.z = -  (((double)m - camera_cy) * p.x / camera_fy);
            p.intensity = intensity_map.at<float>(m, n);
            // p.r = r;
            // p.g = g;
            // p.b = b;
            cloudOut.points.push_back(p);
        }
    }
    return cloudOut.makeShared();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Frame::getFramePointcloud()
{
    return this->frame_pointcloud_.makeShared();
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Frame::getBackgroundcloud()
{
    return this->frame_detections_.cloud_background_.makeShared();
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Frame::getCloud()
{
    return this->frame_detections_.cloud_.makeShared();
}

vector<pcl::PointCloud<pcl::PointXYZI>> * Frame::getObjcloud()
{
    return &frame_detections_.cloud_objects_buffer_;
}


cv::Mat Frame::PointCloudToDepth(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, Config global_config)
{

    double camera_fx = global_config.camera_intrinsic_(0,0);
    double camera_fy = global_config.camera_intrinsic_(1,1);
    double camera_cx = global_config.camera_intrinsic_(0,2);
    double camera_cy = global_config.camera_intrinsic_(1,2);
    int n,m;
    cv::Mat depthImageOut(global_config.imageRows_,
        global_config.imageCols_,CV_16UC1,cv::Scalar::all(0) );
    for(int i = 0;i < cloud_in->size();i++)
    {
        Eigen::Vector4d point_camera(
            cloud_in->points[i].x, 
            cloud_in->points[i].y, 
            cloud_in->points[i].z,
            1.0
        );
        point_camera = global_config.camera_extrinsic_ * point_camera;
        double pointDepth;
        pointDepth = point_camera[2] * global_config.camera_factor_;
        n = ((point_camera[0]) * camera_fx) / point_camera[2] + camera_cx;
        m = ((point_camera[1]) * camera_fy) / point_camera[2] + camera_cy;
        if(m < global_config.imageRows_ 
            && n < global_config.imageCols_ && m > 0 && n > 0)
        {
            if(depthImageOut.at<uint16_t>(m,n))
            {
                pointDepth = std::min((double)depthImageOut.at<uint16_t>(m,n),pointDepth);
            }
            if(pointDepth > 120 * global_config.camera_factor_ 
                || pointDepth < 0.0)
            {
                pointDepth = 0.0;
            }
            if(pointDepth >= 65535) pointDepth = 0;
            depthImageOut.at<uint16_t>(m,n) = pointDepth;
            // std::cout << "depth = " << depthImageOut.at<uint16_t>(m,n) << std::endl;
        }
    }
    return depthImageOut;
}

cv::Mat Frame::PointCloudToDepthWithintensity(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, 
    cv::Mat & intensity_map,
    Config global_config)
{
    double camera_fx = global_config.camera_intrinsic_(0,0);
    double camera_fy = global_config.camera_intrinsic_(1,1);
    double camera_cx = global_config.camera_intrinsic_(0,2);
    double camera_cy = global_config.camera_intrinsic_(1,2);
    int n,m;
    cv::Mat depthImageOut(global_config.imageRows_,
        global_config.imageCols_,CV_16UC1,cv::Scalar::all(0) );
    for(int i = 0;i < cloud_in->size();i++)
    {
        Eigen::Vector4d point_camera(cloud_in->points[i].x, 
                                     cloud_in->points[i].y, 
                                     cloud_in->points[i].z,
                                     1.0);
        point_camera = global_config.camera_extrinsic_ * point_camera;
        double pointDepth;
        pointDepth = point_camera[2] * global_config.camera_factor_;
        n = ((point_camera[0]) * camera_fx) / point_camera[2] + camera_cx;
        m = ((point_camera[1]) * camera_fy) / point_camera[2] + camera_cy;
        if(m < global_config.imageRows_ 
            && n < global_config.imageCols_ && m > 0 && n > 0)
        {
            if(depthImageOut.at<uint16_t>(m,n))
            {
                pointDepth = std::min((double)depthImageOut.at<uint16_t>(m,n),pointDepth);
            }
            if(pointDepth > 120 * global_config.camera_factor_ 
                || pointDepth < 0.0)
            {
                // pointDepth = 0.0;
                continue;
            }
            if(pointDepth >= 65535) 
            {
                // pointDepth = 0;
                continue;
            }
            depthImageOut.at<uint16_t>(m,n) = pointDepth;
            intensity_map.at<float>(m,n) = cloud_in->points[i].intensity;

            // std::cout << "depth = " << depthImageOut.at<uint16_t>(m,n) << std::endl;
            // std::cout << "intensity = " << intensity_map.at<float>(m,n) << std::endl;
        }
    }
    return depthImageOut;
}

void Frame::full_detection(
    pcdsWithTime * pcd_in
)
{
    // 相对于basetime的偏移时间
    double basetime = pcd_in->first / 1000000000.0;
    float offsettime[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
    for (size_t cloudidx = 0; cloudidx < 6; cloudidx++)
    {
        double offset64 = pcd_in->second[cloudidx].first / 1000000000.0;
        offsettime[cloudidx] = offset64 - basetime;
    }

    // 相对于cloud第一个点的偏移时间
    // -----------------get point time----------------- 
    vector<pcl::PointCloud<pcl::PointXYZI>> pcds_xyzi_buffer;
    pcl::PointCloud<pcl::PointXYZI> frame_pcd;
    for (size_t pcd_idx = 0; pcd_idx < pcd_in->second.size(); pcd_idx++)
    {
        pcd_in->second[pcd_idx].second.erase(
            pcd_in->second[pcd_idx].second.begin() + 
            pcd_in->second[pcd_idx].second.size()
        );
        pcd_in->second[pcd_idx].second.erase(
            pcd_in->second[pcd_idx].second.begin() + 
            pcd_in->second[pcd_idx].second.size()
        );
        pcd_in->second[pcd_idx].second.erase(
            pcd_in->second[pcd_idx].second.begin() + 
            pcd_in->second[pcd_idx].second.size()
        );
        pcd_in->second[pcd_idx].second.erase(
            pcd_in->second[pcd_idx].second.begin() + 
            pcd_in->second[pcd_idx].second.size()
        );

        int pointssize = pcd_in->second[pcd_idx].second.size();
        // std::cout << "time \"" << pcd_buffer[frame_idx].second[pcd_idx].first << "\' : " 
        //     << pointssize << std::endl;
        float step_t = 0.1 / (float)pointssize;

        pcl::PointCloud<pcl::PointXYZI> pcd_xyzi;
        for (size_t pointidx = 0; pointidx < pointssize; pointidx++)
        {
            pcl::PointXYZI point_temp;
            point_temp.x = pcd_in->second[pcd_idx].second[pointidx].x;
            point_temp.y = pcd_in->second[pcd_idx].second[pointidx].y;
            point_temp.z = pcd_in->second[pcd_idx].second[pointidx].z;
            // intensity 表示需要偏移的时间 单位s 
            point_temp.intensity = (float)pointidx * step_t + (float)offsettime[pcd_idx];
            pcd_xyzi.push_back(point_temp);
            // std::cout << "time : " << point_temp.intensity << std::endl;
        }
        pcds_xyzi_buffer.push_back(pcd_xyzi);
        frame_pcd += pcd_xyzi;
    }
    frame_detections_.cloud_ = frame_pcd;

    // 背景点云与检测后的目标点云
    frame_detections_.cloud_objects_buffer_.resize(cubes_.size());
    point_extraction(
        frame_detections_.cloud_.makeShared(),
        &cubes_,
        &frame_detections_.cloud_background_,
        &frame_detections_.cloud_objects_buffer_
    );
}

void Frame::point_extraction(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_,
    const frameCubes * cubes,
    pcl::PointCloud<pcl::PointXYZI> * background_cloud,
    vector<pcl::PointCloud<pcl::PointXYZI>> * obj_cloud
)
{
    // 区分点是否在3d检测框内
    for (size_t pt_idx = 0; pt_idx < cloud_->size(); pt_idx++)
    {
        bool is_background = true;
        for (size_t obj_idx = 0; obj_idx < cubes->size(); obj_idx++)
        {
            Eigen::Vector3f pt(
                cloud_->points[pt_idx].x,
                cloud_->points[pt_idx].y,
                cloud_->points[pt_idx].z
            );

            Eigen::Vector3f vec0(
                (*cubes)[obj_idx].cube_vertexs_(0,0) - pt[0],
                (*cubes)[obj_idx].cube_vertexs_(0,1) - pt[1],
                (*cubes)[obj_idx].cube_vertexs_(0,2) - pt[2]
            );

            Eigen::Vector3f vec1(
                (*cubes)[obj_idx].cube_vertexs_(1,0) - pt[0],
                (*cubes)[obj_idx].cube_vertexs_(1,1) - pt[1],
                (*cubes)[obj_idx].cube_vertexs_(1,2) - pt[2]
            );

            Eigen::Vector3f vec3(
                (*cubes)[obj_idx].cube_vertexs_(3,0) - pt[0],
                (*cubes)[obj_idx].cube_vertexs_(3,1) - pt[1],
                (*cubes)[obj_idx].cube_vertexs_(3,2) - pt[2]
            );

            Eigen::Vector3f vec4(
                (*cubes)[obj_idx].cube_vertexs_(4,0) - pt[0],
                (*cubes)[obj_idx].cube_vertexs_(4,1) - pt[1],
                (*cubes)[obj_idx].cube_vertexs_(4,2) - pt[2]
            );

            Eigen::Vector3f nor1(
                (*cubes)[obj_idx].cube_vertexs_(0,0) - (*cubes)[obj_idx].cube_vertexs_(1,0),
                (*cubes)[obj_idx].cube_vertexs_(0,1) - (*cubes)[obj_idx].cube_vertexs_(1,1),
                (*cubes)[obj_idx].cube_vertexs_(0,2) - (*cubes)[obj_idx].cube_vertexs_(1,2)
            );

            Eigen::Vector3f nor3(
                (*cubes)[obj_idx].cube_vertexs_(0,0) - (*cubes)[obj_idx].cube_vertexs_(3,0),
                (*cubes)[obj_idx].cube_vertexs_(0,1) - (*cubes)[obj_idx].cube_vertexs_(3,1),
                (*cubes)[obj_idx].cube_vertexs_(0,2) - (*cubes)[obj_idx].cube_vertexs_(3,2)
            );

            Eigen::Vector3f nor4(
                (*cubes)[obj_idx].cube_vertexs_(0,0) - (*cubes)[obj_idx].cube_vertexs_(4,0),
                (*cubes)[obj_idx].cube_vertexs_(0,1) - (*cubes)[obj_idx].cube_vertexs_(4,1),
                (*cubes)[obj_idx].cube_vertexs_(0,2) - (*cubes)[obj_idx].cube_vertexs_(4,2)
            );

            // 內积乘积为负 
            if (
                cloud_->points[pt_idx].z > -2.0 &&
                nor1.dot(vec0) * nor1.dot(vec1) < 0 &&
                nor3.dot(vec0) * nor3.dot(vec3) < 0 &&
                nor4.dot(vec0) * nor4.dot(vec4) < 0
            )
            {
                is_background = false;
                (*obj_cloud)[obj_idx].push_back(cloud_->points[pt_idx]);
                break;
            }
        }
        if(is_background)
        {
            background_cloud->push_back(cloud_->points[pt_idx]);
        }
    }
}

void Frame::detection_align(
    const vector<pcl::PointCloud<pcl::PointXYZI>> * obj_cloud_,
    const cv::Mat * rawimg_,
    const frameCubes * cubes_,
    const frameBboxs * objs_,
    const Eigen::Matrix4d * global_pose_,
    std::vector<alignedDet> & aligned_detections
)
{
    // 
    vector<pcl::PointCloud<pcl::PointXYZI>> obj_cloud_buffer;
    vector<cv::Rect> proj2dvertex_buffer;

    cv::Mat raw_img = *rawimg_;
    cv::Mat test_img; 
    rawimg_->copyTo(test_img);
    // proj all the 3d detection to 2d domain
    // cubes_ => proj2dvertex_buffer 
    for (size_t obj_idx = 0; obj_idx < cubes_->size(); obj_idx++)
    {
        cv::Rect proj2dvertex;
        detection3dProj2d(
            &(*cubes_)[obj_idx],
            &proj2dvertex
        );
        proj2dvertex_buffer.push_back(proj2dvertex);

        obj_cloud_buffer.push_back((*obj_cloud_)[obj_idx]);

        // cv::rectangle(
        //     test_img, 
        //     proj2dvertex_buffer[obj_idx], 
        //     cv::Scalar(255,0,0),
        //     3, cv::LINE_8, 0
        // );
    }

    vector<vector<double>> iouMatrix;
    iouMatrix.resize(objs_->size(), vector<double>(cubes_->size(), 0));
    for (size_t obj2d_idx = 0; obj2d_idx < objs_->size(); obj2d_idx++)
    {
        for (size_t obj3d_idx = 0; obj3d_idx < cubes_->size(); obj3d_idx++)
        {
            // TODO 两个检测系统的类别可能不同 需再次检查

            float iou_tmp = 0.0;
            // if (
            //     std::strcmp(
            //         (*objs_)[obj2d_idx].object_type_.c_str(),
            //         (*cubes_)[obj3d_idx].object_type_.c_str()
            //     )
            // )
            // {
            //     iouMatrix[obj2d_idx][obj3d_idx] = 1.0;
            //     continue;
            // }
            // if (
            //     (*objs_)[obj2d_idx].score_ < 0.6 ||
            //     (*cubes_)[obj3d_idx].confidence_ < 0.8
            // )
            // {
            //     iouMatrix[obj2d_idx][obj3d_idx] = 1.0;
            //     continue;
            // }
            cv::Rect rect2d(
                (*objs_)[obj2d_idx].bbox_y1_, 
                (*objs_)[obj2d_idx].bbox_x1_,
                (*objs_)[obj2d_idx].bbox_y2_ - (*objs_)[obj2d_idx].bbox_y1_,
                (*objs_)[obj2d_idx].bbox_x2_ - (*objs_)[obj2d_idx].bbox_x1_
            );
            iou_tmp = fusionIoU( 
                proj2dvertex_buffer[obj3d_idx],
                rect2d
            );
            iouMatrix[obj2d_idx][obj3d_idx] = 1.0 - (double)iou_tmp;
            // std::cout << (*objs_)[obj2d_idx].object_type_
            //     << " " << (*cubes_)[obj3d_idx].object_type_
            //     << " " << obj2d_idx
            //     << " " << obj3d_idx
            //     << " " << (*objs_)[obj2d_idx].score_
            //     << " " << (*cubes_)[obj3d_idx].confidence_
            //     << "   iouscore=" << iou_tmp
            //     << std::endl;
        }
    }

    // HungarianAlgorithm 求解最佳2d 3d detection关联
    vector<cv::Point> matchPairs;
    findHungarianAssignment(iouMatrix, matchPairs);

    std::vector<alignedDet> aligneddet_buffer;
    for (size_t pair_idx = 0; pair_idx < matchPairs.size(); pair_idx++)
    {
        

        int idx_2d = matchPairs[pair_idx].x;
        int idx_3d = matchPairs[pair_idx].y;

        obBBOX curbbox = (*objs_)[idx_2d];
        cv::Rect obj_bbox(
            curbbox.bbox_y1_,  
            curbbox.bbox_x1_,
            curbbox.bbox_y2_ - curbbox.bbox_y1_, 
            curbbox.bbox_x2_ - curbbox.bbox_x1_
        );

        int rand_r = (rand() % 255) + 0;
        int rand_g = (rand() % 255) + 0;
        int rand_b = (rand() % 255) + 0;

        alignedDet aligneddet_tmp;
        aligneddet_tmp.type_ = (*cubes_)[idx_3d].object_type_;
        aligneddet_tmp.confidence3d_ = (*cubes_)[idx_3d].confidence_;
        // 3d cube vertexs can not be aligned
        aligneddet_tmp.vertex3d_ = (*cubes_)[idx_3d].cube_vertexs_;
        aligneddet_tmp.vertex2d_ = obj_bbox;
        aligneddet_tmp.confidence2d_ = (*objs_)[idx_2d].score_;

        aligneddet_tmp.cloud_ = (*obj_cloud_)[idx_3d];
        aligneddet_tmp.img_ = *rawimg_;
        aligneddet_tmp.global_pose_ = *global_pose_;
        aligneddet_buffer.push_back(aligneddet_tmp);
    }
    aligned_detections = aligneddet_buffer;
    // the matchpairs size sometimes are smaller than the above twos
}

void Frame::detection3dProj2d(
    const cube3d * vertex3d,
    cv::Rect * output
)
{
    vector<int> projPointsN;
    vector<int> projPointsM;
    double camera_fx = global_config_.camera_intrinsic_(0,0);
    double camera_fy = global_config_.camera_intrinsic_(1,1);
    double camera_cx = global_config_.camera_intrinsic_(0,2);
    double camera_cy = global_config_.camera_intrinsic_(1,2);
    int n,m;
    // 八个点投影到camera平面上
    for (size_t vertex_idx = 0; vertex_idx < vertex3d->cube_vertexs_.rows(); vertex_idx++)
    {
        Eigen::Vector4d point_camera(
            vertex3d->cube_vertexs_(vertex_idx,0),
            vertex3d->cube_vertexs_(vertex_idx,1),
            vertex3d->cube_vertexs_(vertex_idx,2),
            1.0
        );
        point_camera = global_config_.camera_extrinsic_ * point_camera;
        double pointDepth;
        pointDepth = point_camera[2] * global_config_.camera_factor_;
        n = ((point_camera[0]) * camera_fx) / point_camera[2] + camera_cx;
        m = ((point_camera[1]) * camera_fy) / point_camera[2] + camera_cy;
        projPointsN.push_back(n);
        projPointsM.push_back(m);
    }

    // 最大rectangle 四点
    std::sort(projPointsN.begin(), projPointsN.end());
    std::sort(projPointsM.begin(), projPointsM.end());

    output->x = projPointsN[0];
    output->width = projPointsN.back() - projPointsN[0];
    output->y = projPointsM[0];
    output->height = projPointsM.back() - projPointsM[0];
}

float Frame::fusionIoU(
    const cv::Rect detection3d,
    const cv::Rect detection2d
)
{
    float in = (detection3d & detection2d).area();
	float un = detection3d.area() + detection2d.area() - in;

	if (un < DBL_EPSILON)
		return 0;

	return (float)(in / un);
}

void Frame::findHungarianAssignment(
    vector<vector<double>> iouMatrix_,
    vector<cv::Point> & results
)
{
    set<int> allItems;
    set<int> matchedItems;
    set<int> unmatchedDetections;
    set<int> unmatchedTrajectories;
    vector<cv::Point> matchedPairs;

    HungarianAlgorithm HungAlgo;
    vector<int> HungariaAssignment;
    HungAlgo.Solve(iouMatrix_,HungariaAssignment);

    if (iouMatrix_[0].size() > iouMatrix_.size()) //	there are unmatched detections
    {
        for (unsigned int n = 0; n < iouMatrix_[0].size(); n++)
        {
            allItems.insert(n);
        }

        for (unsigned int i = 0; i < iouMatrix_.size(); ++i)
        {
            matchedItems.insert(HungariaAssignment[i]);
        }

        set_difference(allItems.begin(), allItems.end(),
            matchedItems.begin(), matchedItems.end(),
            insert_iterator<set<int>>(unmatchedDetections, unmatchedDetections.begin()));
    }
    // 检测数量少于tracker数量 
    else if (iouMatrix_[0].size() < iouMatrix_.size()) // there are unmatched trajectory/predictions
    {
        for (unsigned int i = 0; i < iouMatrix_.size(); ++i)
        {
            // unassigned label will be set as -1 in the assignment algorithm
            if (HungariaAssignment[i] == -1) 
                unmatchedTrajectories.insert(i);
        }
    }

    for (unsigned int i = 0; i < iouMatrix_.size(); ++i)
    {
        if (HungariaAssignment[i] == -1) // pass over invalid values
            continue;
        if (1 - iouMatrix_[i][HungariaAssignment[i]] < 0.01)
        {
            // std::cout << "1 - iouMatrix[i][HungariaAssignment[i]] = " 
            //     << 1 - iouMatrix_[i][HungariaAssignment[i]] << std::endl;
            unmatchedTrajectories.insert(i);
            unmatchedDetections.insert(HungariaAssignment[i]);
        }
        else
            matchedPairs.push_back(cv::Point(i, HungariaAssignment[i]));
    }
    results = matchedPairs;

}

Frame::~Frame()
{
}


VisHandel::VisHandel(ros::NodeHandle nh)
{
    nh_ = nh;

    raw_obj_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/raw_obj_cloud", 1000);
    undistorted_obj_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/undistorted_obj_cloud", 1000);
    raw_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/raw_cloud", 1000);
    background_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/background_cloud", 1000);

    raw_img_ = nh_.advertise<sensor_msgs::Image>("/raw_img", 1000);
    img_detection_ = nh_.advertise<sensor_msgs::Image>("/img_detection", 1000);
    marker_3d_ = nh_.advertise<visualization_msgs::Marker>("/marker_3d", 1000);
    txt_marker_3d_ = nh_.advertise<visualization_msgs::Marker>("/txt_marker_3d", 1000);

    obj_velocity_txt_ = nh_.advertise<visualization_msgs::MarkerArray>("/obj_velocity_txt", 1000);
    obj_velocity_arrow_ = nh_.advertise<visualization_msgs::Marker>("/obj_velocity_arrow", 1000);
}

VisHandel::~VisHandel()
{
}

void VisHandel::txt_marker_3d_publisher(
    const std::vector<alignedDet> & detection_in
)
{
    Eigen::Matrix<int, 12, 2> lines;
    lines << 
        0, 1,
        1, 2,
        2, 3,
        3, 0,
        4, 5, 
        5, 6,
        6, 7, 
        7, 4, 
        0, 4, 
        1, 5, 
        2, 6,
        3, 7;
    visualization_msgs::Marker cube_3d_msg;
    cube_3d_msg.header.frame_id = "livox";
    cube_3d_msg.action = visualization_msgs::Marker::ADD;
    cube_3d_msg.type = visualization_msgs::Marker::LINE_LIST;
    cube_3d_msg.scale.x = 0.1;
    cube_3d_msg.color.r = 1.0;
    cube_3d_msg.color.a = 1.0;
    cube_3d_msg.lifetime = ros::Duration(0);

    // all detection cubes 
    for (size_t ob_idx = 0; ob_idx < detection_in.size(); ob_idx++)
    {
        for (size_t line_idx = 0; line_idx < lines.rows(); line_idx++)
        {
            geometry_msgs::Point point1;
            point1.x = detection_in[ob_idx].vertex3d_(lines(line_idx, 0),0);
            point1.y = detection_in[ob_idx].vertex3d_(lines(line_idx, 0),1);
            point1.z = detection_in[ob_idx].vertex3d_(lines(line_idx, 0),2);
            geometry_msgs::Point point2;
            point2.x = detection_in[ob_idx].vertex3d_(lines(line_idx, 1),0);
            point2.y = detection_in[ob_idx].vertex3d_(lines(line_idx, 1),1);
            point2.z = detection_in[ob_idx].vertex3d_(lines(line_idx, 1),2);
            cube_3d_msg.points.push_back(point1);
            cube_3d_msg.points.push_back(point2);
        }
    }

    marker_3d_.publish(cube_3d_msg);
}

void VisHandel::obj_vel_arrow_publisher(visualization_msgs::Marker arrow_in)
{
    obj_velocity_arrow_.publish(arrow_in);
}

void VisHandel::obj_vel_txt_publisher(visualization_msgs::MarkerArray arrow_in)
{
    obj_velocity_txt_.publish(arrow_in);
}

void VisHandel::raw_cloud_publisher(sensor_msgs::PointCloud2 cloud_in)
{
    cloud_in.header.frame_id = "livox";
    raw_cloud_.publish(cloud_in);
}

void VisHandel::background_cloud_publisher(sensor_msgs::PointCloud2 cloud_in)
{
    cloud_in.header.frame_id = "livox";
    background_cloud_.publish(cloud_in);
}

void VisHandel::raw_obj_cloud_publisher(sensor_msgs::PointCloud2 cloud_in)
{
    cloud_in.header.frame_id = "livox";
    raw_obj_cloud_pub_.publish(cloud_in);
}

void VisHandel::undistorted_obj_cloud_publisher(sensor_msgs::PointCloud2 cloud_in)
{
    cloud_in.header.frame_id = "livox";
    undistorted_obj_cloud_pub_.publish(cloud_in);
}

void VisHandel::raw_img_publisher(cv::Mat img_in)
{
    sensor_msgs::ImagePtr img_msg = 
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_in).toImageMsg();
    img_msg->header.frame_id = "livox";

    raw_img_.publish(*img_msg);
}

void VisHandel::label_img_publisher(cv::Mat img_in)
{
    sensor_msgs::ImagePtr img_msg = 
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_in).toImageMsg();
    img_msg->header.frame_id = "livox";

    img_detection_.publish(*img_msg);
}