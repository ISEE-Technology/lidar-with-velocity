#include "ros/ros.h"
#include <cstdlib>

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "add_two_ints_client");

//     ros::NodeHandle n;
//     ros::ServiceClient client = 
//         n.serviceClient<object_undistort::pcsrv>("gnd_estimate_service");
//     object_undistort::pcsrv srv;
//     srv.request.a = int64_t(3);
//     // srv.request.b = atoll(argv[2]);

//     ros::Time start_time = ros::Time::now(); 
//     if (client.call(srv))
//     {
//         ROS_INFO("response : %ld", (long int)srv.response.b);
//         std::cout << "cost time = " << ros::Time::now() - start_time << std::endl;
//     }
//     else
//     {
//         ROS_ERROR("Failed to call service add_two_ints");
//         return 1;
//     }

//     return 0;
// }