#include "lidar-slam/System.h"


System::System(std::string scan_topic_i,std::string map_topic_o,std::string pcl_topic_o,std::string path_topic_o,ros::NodeHandle& nh)
{
    //保存相应的话题消息名称
    m_topic_scan_i_ = scan_topic_i;
    m_topic_map_o_ = map_topic_o;
    m_topic_pcl_o_ = pcl_topic_o;
    m_topic_path_o_ = path_topic_o;

    m_nh_ = nh;

    //初始化话题发布者，订阅者需要确定系统模式后才能初始化
    m_map_pub_ = m_nh_.advertise<nav_msgs::OccupancyGrid>(m_topic_map_o_,1,true);
    m_pcl_pub_ = m_nh_.advertise<sensor_msgs::PointCloud>(m_topic_pcl_o_,1,true);
    m_path_pub_ = m_nh_.advertise<nav_msgs::Path>(m_topic_path_o_,1,true);
}

System::~System()
{

}

//设置测试模式
void System::SetTestMode(std::string bagfile){
    m_bag_file_ = bagfile;
    m_is_test_ = true;
}

void System::Run(){

    if(m_is_test_){
        std::cout << "欢迎进入测试模式" <<std::endl;


    }else{


    }
}