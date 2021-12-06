#include "common/main.h"

assignmentDetector::assignmentDetector(int argc, char** argv)
{
    if(!config_.readParam())
    {
        std::cout << "ERROR! read param fail!" << std::endl;
        exit(1);
    }

    run(argc, argv);
}

assignmentDetector::~assignmentDetector(){}

bool assignmentDetector::run(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_with_velocity");
    ros::NodeHandle nh;
    VisHandel vis(nh);

    // raw imgs 
    std::vector<cv::String> raw_img_fn;
    cv::glob(config_.raw_img_path_, raw_img_fn, false);
    if (raw_img_fn.size() == 0)
    {
        throw("raw img path doesn't exist !");
    }
    else
    {
        std::cout << "find raw img : " << raw_img_fn.size() << std::endl;
    }
    

    // label imgs
    std::vector<cv::String> label_img_fn;
    cv::glob(config_.label_img_path_, label_img_fn, false);
    if (label_img_fn.size() == 0)
    {
        throw("label img path doesn't exist !");
    }
    else
    {
        std::cout << "find label img : " << label_img_fn.size() << std::endl;
    }
    

    // .pcd pointcloud files  
    std::vector<std::string> allFileName = getFilesList(config_.pcd_path_, false);
    if (allFileName.size() == 0)
    {
        throw("pcd path doesn't exist !");
    }
    else
    {
        std::cout << "find pcd : " << allFileName.size() << std::endl;
    }

    // 2d detection bbox files 
    std::vector<std::string> bboxAllFileName = getFilesList(config_.bbox_path_, false);
    if (bboxAllFileName.size() == 0)
    {
        throw("bbox path doesn't exist !");
    }
    else
    {
        std::cout << "find bbox : " << bboxAllFileName.size() << std::endl;
    }

    // 3d detection cube files
    std::vector<std::string> detection3dFileName = getFilesList(config_.cube_path_, false);
    if (detection3dFileName.size() == 0)
    {
        throw("3d detection path doesn't exist !");
    }
    else
    {
        std::cout << "find cube : " << detection3dFileName.size() << std::endl << std::endl;
    }

    // 3d detection cube files 
    Timer detection_3d_timer("detection_3d_timer");
    std::vector<frameCubesWithTime> detection_3d_buffer;
    for (size_t detection_3d_idx = 0; detection_3d_idx < detection3dFileName.size(); 
        detection_3d_idx++)
    {
        std::string detection_3d_path = detection3dFileName[detection_3d_idx];
        std::string detection_3d_name = detection_3d_path;
        detection_3d_name.erase(0,  config_.cube_path_.size());
        detection_3d_name.erase(detection_3d_name.size() - 5, detection_3d_name.size());
        YAML::Node detection_3d_data = YAML::LoadFile(detection_3d_path);
        int obj_size;
        frameCubes current_cubes;
        if (detection_3d_data["ob_num"]) 
        {
            obj_size = detection_3d_data["ob_num"].as<int>();
        }
        for (size_t ob_idx = 0; ob_idx < obj_size; ob_idx++)
        {
            std::string ob_name = "ob" + std::to_string(ob_idx + 1);
            std::string object_type;
            double confidence;
            Eigen::Matrix<double, 8, 3> cube_vertexs;
            if (detection_3d_data[ob_name]["class"]) 
            {
                object_type = detection_3d_data[ob_name]["class"].as<std::string>();
            }
            if (detection_3d_data[ob_name]["confidence"]) 
            {
                confidence = detection_3d_data[ob_name]["confidence"].as<double>();
            }
            if (detection_3d_data[ob_name]["points"]) 
            {
                std::vector<double> vertexs_data = 
                    detection_3d_data[ob_name]["points"].as<std::vector<double>>();
                double* vertexs_array = vertexs_data.data();
                cube_vertexs = Eigen::Map<Eigen::Matrix<double, 3, 8>>(vertexs_array).transpose();
                cube3d detected_cube(
                    object_type,
                    confidence,
                    cube_vertexs
                );
                current_cubes.push_back(detected_cube);
            }
        }
        frameCubesWithTime detection_cube(std::atoi(detection_3d_name.c_str()), current_cubes);
        detection_3d_buffer.push_back(detection_cube);
    }
    std::cout << "\nreaded detection 3d files : " << detection_3d_buffer.size() << ", ";
    detection_3d_timer.rlog("detection_3d_timer cost");
    cout << "\n";

    // raw imgs 
    Timer raw_reader_timer("raw_reader_timer");
    std::vector<imageWithTime> raw_img_buffer;
    for (size_t raw_img_idx = 0; raw_img_idx < raw_img_fn.size(); raw_img_idx++)
    {
        std::string raw_img_time_str = raw_img_fn[raw_img_idx];
        raw_img_time_str 
            = raw_img_time_str.erase(0, config_.raw_img_path_.size()).erase(10,1).erase(19,4);
        uint64_t raw_img_time_uint64;
        raw_img_time_uint64 = 
            std::strtoull(raw_img_time_str.c_str(), NULL, 0);
        std::cout << "\rreading raw img : " << raw_img_time_uint64 << ",  "
            << raw_img_idx+1 << "/" << raw_img_fn.size();
        cv::Mat current_raw_img;
        current_raw_img = cv::imread(raw_img_fn[raw_img_idx]);
        imageWithTime raw_img_and_time(raw_img_time_uint64,
            current_raw_img);
        raw_img_buffer.push_back(raw_img_and_time);
    }
    std::cout << "\nreaded raw img : " << raw_img_buffer.size() << ", ";
    raw_reader_timer.rlog("raw_reader_timer cost");
    std::cout << "\n";

    // label imgs
    Timer label_reader_timer("label_reader_timer");
    std::vector<imageWithTime> label_img_buffer;
    for (size_t label_img_idx = 0; label_img_idx < label_img_fn.size(); 
        label_img_idx++)
    {
        std::string label_img_time_str = label_img_fn[label_img_idx];
        label_img_time_str 
            = label_img_time_str.erase(0, 
            config_.label_img_path_.size()).erase(10,1).erase(19,4);
        uint64_t label_img_time_uint64;
        label_img_time_uint64 = 
            std::strtoull(label_img_time_str.c_str(), NULL, 0);
        std::cout << "\rreading label img : " << label_img_time_uint64 << ",  "
            << label_img_idx+1 << "/" << label_img_fn.size();
        cv::Mat current_label_img;
        current_label_img = cv::imread(label_img_fn[label_img_idx]);
        imageWithTime label_img_and_time(label_img_time_uint64,
            current_label_img);
        label_img_buffer.push_back(label_img_and_time);
    }
    std::cout << "\nreaded label img : " << label_img_buffer.size() << ", ";
    label_reader_timer.rlog("label_reader_timer cost");
    std::cout << "\n";

    // 2d detection bbox files 
    Timer bbox_reader_timer("bbox_reader_timer");
    std::vector<frameBboxsWithTime> bbox_buffer;
    for (size_t framebboxs_idx = 0; framebboxs_idx < bboxAllFileName.size(); 
        framebboxs_idx++)
    {
        frameBboxs current_frame_obj;
        std::string frameBboxs_path = bboxAllFileName[framebboxs_idx];
        std::string bboxs_time_str = bboxAllFileName[framebboxs_idx];
        bboxs_time_str = bboxs_time_str.erase(0,
            bboxs_time_str.size() - 25).erase(20, 5).erase(10, 1);
        uint64_t bboxs_time_uint64;
        bboxs_time_uint64 = std::strtoull(bboxs_time_str.c_str(), NULL, 0);
        YAML::Node bboxs_data = YAML::LoadFile(frameBboxs_path);
        int obj_size;
        if (bboxs_data["ob_num"]) 
        {
            obj_size = bboxs_data["ob_num"].as<int>();
            obj_size -= 1;
        }
        for (size_t ob_idx = 0; ob_idx < obj_size; ob_idx++)
        {
            std::string ob_name = "ob" + std::to_string(ob_idx + 1);
            std::string object_type;
            double score;
            int bbox_x1;
            int bbox_x2;
            int bbox_y1;
            int bbox_y2;
            if (bboxs_data[ob_name]["object_name"]) 
            {
                object_type = bboxs_data[ob_name]["object_name"].as<std::string>();
            }
            if (bboxs_data[ob_name]["confidence_score"]) 
            {
                score = bboxs_data[ob_name]["confidence_score"].as<double>();
            }
            if (bboxs_data[ob_name]["bbox_x1"]) 
            {
                bbox_x1 = bboxs_data[ob_name]["bbox_x1"].as<int>();
            }
            if (bboxs_data[ob_name]["bbox_x2"]) 
            {
                bbox_x2 = bboxs_data[ob_name]["bbox_x2"].as<int>();
            }
            if (bboxs_data[ob_name]["bbox_y1"]) 
            {
                bbox_y1 = bboxs_data[ob_name]["bbox_y1"].as<int>();
            }if (bboxs_data[ob_name]["bbox_y2"]) 
            {
                bbox_y2 = bboxs_data[ob_name]["bbox_y2"].as<int>();
            }
            obBBOX object(
                object_type,
                score,
                bbox_y1,
                bbox_y2,
                bbox_x1,
                bbox_x2
            );
            current_frame_obj.push_back(object);
        }
        frameBboxsWithTime current_frame_with_time(bboxs_time_uint64, 
            current_frame_obj);
        bbox_buffer.push_back(current_frame_with_time);
    }
    std::cout << "readed Bbox : " << bbox_buffer.size() << ", ";
    bbox_reader_timer.rlog("bbox_reader_timer cost");
    std::cout << "\n";

    // .pcd pointcloud files  
    Timer pcd_reader_timer("pcd_reader_timer");
    std::vector<pcdsWithTime> pcd_buffer;
    std::vector<pair<uint64_t, Eigen::Matrix4d>> pose_buffer;
    YAML::Node pose_config = YAML::LoadFile(
        this->config_.pose_path_
    );
    for (size_t pcd_idx = 0; pcd_idx < allFileName.size(); pcd_idx++)
    {
        std::vector<std::string> frame_pcds = 
            getFilesList(allFileName[pcd_idx], false);
        pcl::PointCloud<pcl::PointXYZRGB> current_point_cloud[hubLidarNum];
        std::string pcds_time_str = allFileName[pcd_idx];
        std::string pose_time = pcds_time_str.erase(0,pcds_time_str.size()-20);
        pcds_time_str =  pcds_time_str.erase(
                0,pcds_time_str.size()-20).erase(10,1);
        uint64_t pcds_time_uint64;
        pcds_time_uint64 = std::strtoull(pcds_time_str.c_str(), NULL, 0);
        pcdWithTime current_pcds;
        Eigen::Matrix4d pose_matrix;
        if (pose_config[pose_time])
        {
            std::vector<double> pose_vector = 
                pose_config[
                    pose_time
                ].as<std::vector<double>>();
            double* pose_array = pose_vector.data();
            pose_matrix = 
                Eigen::Map<Eigen::Matrix4d>(pose_array).transpose();
        }
        else
        { 
            return false;
        }
        for (size_t lidar_idx = 0; lidar_idx < frame_pcds.size(); lidar_idx++)
        {
            std::string pcd_time_str = frame_pcds[lidar_idx];
            pcd_time_str =  pcd_time_str.erase(
                    0,pcd_time_str.size()-23).erase(19,4);
            uint64_t pcd_time_uint64;
            pcd_time_uint64 = std::strtoull(pcd_time_str.c_str(), NULL, 0);
            pcl::io::loadPCDFile(frame_pcds[lidar_idx], 
                    current_point_cloud[lidar_idx]);
            std::cout << "\rreading pcd : " << pcds_time_uint64 << ",  "
                << lidar_idx+1 << "/" << frame_pcds.size() << ",  "
                << pcd_idx+1 << "/" << allFileName.size();
            current_pcds.push_back(
                std::pair<uint64_t, pcl::PointCloud<pcl::PointXYZRGB>>(
                pcd_time_uint64, current_point_cloud[lidar_idx]
                    ));
        }
        pcdsWithTime pcds_and_time(pcds_time_uint64, current_pcds);
        pcd_buffer.push_back(pcds_and_time);
        pose_buffer.push_back(pair<uint64_t, Eigen::Matrix4d>(pcds_time_uint64,
            pose_matrix));
    }
    std::cout << "\nreaded pcd : " << pcd_buffer.size() << ", ";
    pcd_reader_timer.rlog("pcd_reader_timer cost");
    std::cout << "\n";

    // global poses
    std::cout << "readed pose : " << pose_buffer.size() << "\n\n";


    // sort the input imgs and pcds by timestamp
    if(pcd_buffer.size() == raw_img_buffer.size() 
        && pcd_buffer.size() == label_img_buffer.size())
    {
        std::sort(raw_img_buffer.begin(), raw_img_buffer.end(), imgSort);
        std::sort(label_img_buffer.begin(), label_img_buffer.end(), imgSort);
        std::sort(pcd_buffer.begin(), pcd_buffer.end(), pcdSort);
        std::sort(bbox_buffer.begin(), bbox_buffer.end(), bboxSort);
        std::sort(detection_3d_buffer.begin(), detection_3d_buffer.end(), cubeSort);
        std::sort(pose_buffer.begin(), pose_buffer.end(), poseSort);
    }
    else
    {
        throw("ERROR! input imgs and pcds don't match!");
    }

    // vis tool initialization 
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_objs
        (new pcl::visualization::PCLVisualizer("3D Viewer objects"));
    viewer_objs->setBackgroundColor(0, 0, 0);
    viewer_objs->addCoordinateSystem(0.5);
    viewer_objs->setCameraPosition(-46.698877,8.333347,39.880589,0.449239,-0.004796,0.893399);

    // main frame loop
    fusion_tracker fusionTracker;
    size_t loop_count = 0;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> final_last_pcs;
    visualization_msgs::MarkerArray obj_vel_txt_markerarray;
    for (size_t frame_idx = 0; frame_idx < pcd_buffer.size(); ++frame_idx)
    {
        Timer frame_timer("This frame time");

        cout << "=========================== seq:" << 
            frame_idx + 1 << " ===========================" << endl;

        expand_3d_cube(detection_3d_buffer[frame_idx]);

        Frame frame(
            raw_img_buffer[frame_idx],
            label_img_buffer[frame_idx],
            pcd_buffer[frame_idx],      
            bbox_buffer[frame_idx],
            detection_3d_buffer[frame_idx],
            config_
        );

        frame.full_detection(&pcd_buffer[frame_idx]);
        // frame.verboseFrame();

        std::vector<alignedDet> aligned_detection_buffer;
        std::vector<pcl::PointCloud<pcl::PointXYZI>> * obj_clouds;
        pcl::PointCloud<pcl::PointXYZI> obj_cloud;
        obj_clouds = frame.getObjcloud();
        // no detection obj order !
        frame.detection_align(
            obj_clouds,
            &raw_img_buffer[frame_idx].second,
            &detection_3d_buffer[frame_idx].second,
            &bbox_buffer[frame_idx].second,
            &pose_buffer[frame_idx].second,
            aligned_detection_buffer
        );

        vis.txt_marker_3d_publisher(aligned_detection_buffer);

        // raw cloud
        sensor_msgs::PointCloud2 raw_cloud_msg;
        pcl::PointCloud<pcl::PointXYZI> frame_pcd = *frame.getCloud(); 
        pcl::toROSMsg(frame_pcd, raw_cloud_msg);
        viewer_objs->addPointCloud<pcl::PointXYZI>(
            frame_pcd.makeShared(), 
            "background_cloud" + std::to_string(frame_idx + 1)
        );
        vis.raw_cloud_publisher(raw_cloud_msg);

        // background cloud
        sensor_msgs::PointCloud2 background_cloud_msg;
        pcl::PointCloud<pcl::PointXYZI> background_cloud = *frame.getBackgroundcloud(); 
        pcl::toROSMsg(background_cloud, background_cloud_msg);
        vis.background_cloud_publisher(background_cloud_msg);

        // obj cloud
        for (size_t pcd_idx = 0; pcd_idx < aligned_detection_buffer.size(); pcd_idx++)
        {
            obj_cloud += aligned_detection_buffer[pcd_idx].cloud_;
        }
        sensor_msgs::PointCloud2 obj_cloud_msg;
        pcl::toROSMsg(obj_cloud, obj_cloud_msg);
        vis.raw_obj_cloud_publisher(obj_cloud_msg);

        vis.raw_img_publisher(raw_img_buffer[frame_idx].second);

        vis.label_img_publisher(label_img_buffer[frame_idx].second);

        fusionTracker.tracking(
            aligned_detection_buffer,
            raw_img_buffer[frame_idx].second,
            config_,
            viewer_objs,
            obj_vel_txt_markerarray,
            &vis
        );

        frame_timer.rlog("This frame cost time");

        loop_count++;
        while (1) 
        {
            is_nextFrame_ = false;

            viewer_objs->spinOnce(1);

            viewer_objs->registerKeyboardCallback(&nextFrame, &is_nextFrame_);

            cv::waitKey(1);

            if(is_nextFrame_)
            {
                // viewer_objs->removeAllShapes();
                viewer_objs->removeAllPointClouds();

                break;
            }
        }
    }
}

void nextFrame(const pcl::visualization::KeyboardEvent& event, void* val) 
{
    bool *pKey = (bool *)val;
    if (event.keyDown()) 
    {
        if(strcmp(event.getKeySym().c_str(), "f"))
        {
            *pKey = true;
        }
        else
        {
            *pKey = false;
        }
        
    }
}

std::vector<std::string> getFilesList(std::string dirpath, bool is_recursive)
{
    DIR *dir = opendir(dirpath.c_str());
    if (dir == NULL)
    {
        std::cout << "opendir error" << std::endl;
    }

    std::vector<std::string> allPath;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL)
    {
        // if (entry->d_type == DT_DIR)
        if(is_recursive)
        {//It's dir
            if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
                continue;
            std::string dirNew;
            if(std::strcmp(&(dirpath[dirpath.length()-1]), "/") != 0)
            {
                dirNew = dirpath + "/" + entry->d_name;
            }
            else
            {
                dirNew = dirpath + entry->d_name;
                
            }
            std::vector<std::string> tempPath = getFilesList(dirNew, 
                !entry->d_type == DT_DIR);
            allPath.insert(allPath.end(), tempPath.begin(), tempPath.end());

        }
        else 
        {
            if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
                continue;
            std::string name = entry->d_name;
            std::string imgdir;
            if(std::strcmp(&(dirpath[dirpath.length()-1]), "/") != 0)
            {
                imgdir = dirpath + "/" + name;
            }
            else
            {
                imgdir = dirpath + name;
            }
            allPath.push_back(imgdir);
        }

    }
    closedir(dir);
    return allPath;
}

bool imgSort(const imageWithTime& left, const imageWithTime& right)
{
    return (left.first < right.first);
}

bool pcdSort(const pcdsWithTime& left, const pcdsWithTime& right)
{
    return (left.first < right.first);
}
bool bboxSort(const frameBboxsWithTime& left, const frameBboxsWithTime& right)
{
    return (left.first < right.first);
}
bool cubeSort(const frameCubesWithTime& left, const frameCubesWithTime& right)
{
    return (left.first < right.first);
}
bool poseSort(const pair<uint64_t, Eigen::Matrix4d>& left, 
    const pair<uint64_t, Eigen::Matrix4d>& right)
{
    return (left.first < right.first);
}
bool pcSort(const pcl::PointCloud<pcl::PointXYZI>& left, 
    const pcl::PointCloud<pcl::PointXYZI>& right)
{
    return (left.size() < right.size());
}

double GetIOU(Cube bb_test, Cube bb_gt)
{
    /* a 2d projection iou method */
    cv::Rect_<double> box1(
        bb_test.centerx_+0.5*bb_test.depth_,
        bb_test.centery_+0.5*bb_test.width_,
        bb_test.depth_,
        bb_test.width_
    );
    cv::Rect_<double> box2(
        bb_gt.centerx_+0.5*bb_gt.depth_,
        bb_gt.centery_+0.5*bb_gt.width_,
        bb_gt.depth_,
        bb_gt.width_
    );
    float in = (box1 & box2).area();
	float un = box1.area() + box2.area() - in;

	if (un < DBL_EPSILON)
		return 0;

	return (double)(in / un);

}

void expand_3d_cube(
    frameCubesWithTime & cubes_in
)
{
    for (size_t obj_idx = 0; obj_idx < cubes_in.second.size(); obj_idx++)
    {
        Eigen::Vector3d obj_center = cubes_in.second[obj_idx].cube_vertexs_.colwise().mean();
        for (size_t pt_idx = 0; pt_idx < 8; pt_idx++)
        {
            Eigen::Vector3d expand_value;
            double expand_ratio = 0.2;
            expand_value[0] = expand_ratio * (cubes_in.second[obj_idx].cube_vertexs_(pt_idx,0) - obj_center[0]);
            expand_value[1] = expand_ratio * (cubes_in.second[obj_idx].cube_vertexs_(pt_idx,1) - obj_center[1]);
            expand_value[2] = expand_ratio * (cubes_in.second[obj_idx].cube_vertexs_(pt_idx,2) - obj_center[2]);

            cubes_in.second[obj_idx].cube_vertexs_(pt_idx,0) += expand_value[0];
            cubes_in.second[obj_idx].cube_vertexs_(pt_idx,1) += expand_value[1];
            cubes_in.second[obj_idx].cube_vertexs_(pt_idx,2) += expand_value[2];
        }
    }
    
}