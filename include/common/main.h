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

template <typename T>
bool timestampSort(const T & left, const T & right)
{
    return (left.first < right.first);
}

void nextFrame(const pcl::visualization::KeyboardEvent& event, void* nothing); 
void getFilesList(const std::string & dirpath, 
    const bool & is_recursive, std::vector<std::string> & out_filelist);
double GetIOU(const Cube & bb_test,const Cube & bb_gt);
void expand_3d_detection(
    frameCubesWithTime & cubes_in
);



class AssignmentDetector
{
private:
    Config config_;

public:
    AssignmentDetector(int argc, char** argv);
    ~AssignmentDetector();

    bool run(int argc, char** argv);
    bool is_nextFrame_ = false;
};


#endif //MAIN_H