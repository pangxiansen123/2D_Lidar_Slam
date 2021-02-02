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

//系统运行函数
void System::Run(){

    if(m_is_test_){
        //! 测试消息
        std::cout << "欢迎进入测试模式" <<std::endl;
        //读取rosbag文件
        rosbag::Bag bag;
        bag.open(m_bag_file_, rosbag::bagmode::Read);

        //添加要读取的消息
        std::vector<std::string> topics;
        topics.push_back(std::string(m_topic_scan_i_));
        rosbag::View view(bag, rosbag::TopicQuery(topics));

        //按顺序读取bag内激光的消息和里程计的消息
        //! 测试消息
        std::cout << "开始循环读取rosbag内消息" << std::endl;
        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            sensor_msgs::MultiEchoLaserScanConstPtr scan = m.instantiate<sensor_msgs::MultiEchoLaserScan>();
            if(scan != NULL)
                LaserScanCallback(scan);

            if(!ros::ok())
                break;
        }
    }else{


    }
}

//激光雷达数据回调函数
void System::LaserScanCallback(sensor_msgs::MultiEchoLaserScanConstPtr& msg){
    sensor_msgs::MultiEchoLaserScan scan = *msg;
    //! 测试消息
    //std::cout << scan.header.stamp << std::endl;

    

}