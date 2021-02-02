#ifndef SYSTEM_H
#define SYSTEM_H
#include <string>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Path.h>

class System
{
private:
    /* data */
    //相关的话题名称
    std::string m_topic_scan_i_;
    std::string m_topic_map_o_;
    std::string m_topic_pcl_o_;
    std::string m_topic_path_o_;

    //相关的话题发布者和订阅者
    ros::Subscriber m_scan_sub_;
    ros::Publisher m_map_pub_;
    ros::Publisher m_pcl_pub_;
    ros::Publisher m_path_pub_;

    //ros句柄
    ros::NodeHandle m_nh_;

    //是否使用bag包测试系统及bag包地址
    bool m_is_test_ = false;
    std::string m_bag_file_;


public:
    System(std::string scan_topic_i,std::string map_topic_o,std::string pcl_topic_o,std::string path_topic_o,ros::NodeHandle& nh );
    ~System();
    void SetTestMode(std::string bagfile);
    void Run();
};









#endif