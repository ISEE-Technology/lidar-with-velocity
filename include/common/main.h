//main.h
#ifndef MAIN_H
#define MAIN_H

#include <iostream> 
#include <fstream>
#include <stdlib.h>
#include <mutex>
#include <termio.h>
#include <stdio.h> 
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "tracker/tracker.h"
#include "common/frame.h"

void nextFrame(const pcl::visualization::KeyboardEvent& event, void* nothing); 
std::vector<std::string> getFilesList(std::string dirpath, bool is_recursive);
bool imgSort(const imageWithTime& left, const imageWithTime& right);
bool pcdSort(const pcdsWithTime& left, const pcdsWithTime& right);
bool bboxSort(const frameBboxsWithTime& left, const frameBboxsWithTime& right);
bool cubeSort(const frameCubesWithTime& left, const frameCubesWithTime& right);
bool poseSort(
    const pair<uint64_t, Eigen::Matrix4d>& left, 
    const pair<uint64_t, Eigen::Matrix4d>& right);
bool pcSort(const pcl::PointCloud<pcl::PointXYZI>& left, 
    const pcl::PointCloud<pcl::PointXYZI>& right);
double GetIOU(Cube bb_test, Cube bb_gt);
void expand_3d_cube(
    frameCubesWithTime & cubes_in
);

class assignmentDetector
{
private:
    Config config_;

public:
    assignmentDetector(int argc, char** argv);
    ~assignmentDetector();

    bool run(int argc, char** argv);
    bool is_nextFrame_ = false;
};


#endif //MAIN_H