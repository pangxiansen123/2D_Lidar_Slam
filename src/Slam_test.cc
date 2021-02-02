#include "lidar-slam/System.h"
#include <ros/ros.h>


int main(int argc,char ** argv){

    ros::init(argc,argv,"slam_test");
    ros::NodeHandle n;
    std::string scan = "horizontal_laser_2d";
    std::string path = "Path";
    std::string pcl = "Pcl";
    std::string map = "Map";
    std::string bagfile  = "/home/teamo/lidar-slam/src/Data/test.bag";

    System system(scan,map,pcl,path,n);
    system.SetTestMode(bagfile);
    system.Run();
    ros::spin();
    return 0;

}