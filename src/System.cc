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

    //
    m_is_first_frame_ = true;

    //初始化地图
    m_path_.header.frame_id = "odom";
    m_path_.header.stamp = ros::Time::now();

    //初始化匹配器
    pt_matcher_ = new Matcher();

    //初始化地图
    MapParams params;
    params.width = 1000;
    params.height = 1000;
    params.resolution = 0.05;
    params.log_occ = 2;
    params.log_free = -1;
    params.log_min = 0;
    params.log_max = 100;
    params.offset_x = 500;
    params.offset_y = 500;
    params.origin_x = 0.0;
    params.origin_y = 0.0;

    pt_mapper_ = new OccupanyMapper(params);
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

void System::PubPath(Eigen::Vector3d& pose, nav_msgs::Path &path, ros::Publisher &mcu_path_pub_){
        
        ros::Time current_time = ros::Time::now();
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = pose(0);
        this_pose_stamped.pose.position.y = pose(1);

        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(pose(2));
        this_pose_stamped.pose.orientation.x = goal_quat.x;
        this_pose_stamped.pose.orientation.y = goal_quat.y;
        this_pose_stamped.pose.orientation.z = goal_quat.z;
        this_pose_stamped.pose.orientation.w = goal_quat.w;

        this_pose_stamped.header.stamp = current_time;
        this_pose_stamped.header.frame_id = "odom";
        path.poses.push_back(this_pose_stamped);
        mcu_path_pub_.publish(path);
}



//激光雷达数据回调函数
void System::LaserScanCallback(sensor_msgs::MultiEchoLaserScanConstPtr& msg){
    sensor_msgs::MultiEchoLaserScan scan = *msg;
    //! 测试消息
    //std::cout << scan.header.stamp << std::endl;

    if(m_is_first_frame_){
        std::cout << "接收到第一帧激光数据，系统初始化" <<std::endl;

        //初始化scan_id
        m_scan_id_ = 0;

        //将当前帧的激光扫描转换成激光坐标系下的局部点云
        TOOLS::TransLaserscanTolocalpts(scan,m_local_pts_);
        //! 测试消息
        //std::cout << "有效初始点云数:" << m_local_pts_.size() << std::endl;

        //将第一帧初始化为坐标系原点
        m_poses_.push_back(Eigen::Vector3d(0.0,0.0,0.0));

        //发布路径
        PubPath(m_poses_[m_scan_id_],m_path_,m_path_pub_);

        pt_matcher_->setTargetPointCloud(m_local_pts_);

        //更新全局坐标点
        TOOLS::Translocaltompts(m_local_pts_,m_pts_,m_poses_[m_scan_id_]);

        //建图
        pt_mapper_->Mapping(m_pts_,m_poses_[m_scan_id_]);

        m_is_first_frame_ = false;
        m_scan_id_++;
        return;
    }


    //将当前扫描数据转换为激光坐标系下的点云
    std::vector<Eigen::Vector2d> now_local_pts;
    TOOLS::TransLaserscanTolocalpts(scan,now_local_pts);

    //设置当前帧和前一帧的点云
    pt_matcher_->setSourcePointCloud(now_local_pts);

    Eigen::Matrix3d T_nowtopre,rCovariance;
    if(pt_matcher_->Match(T_nowtopre,rCovariance,Eigen::Matrix3d().setZero())){
        
        std::cout <<"IMLS Match Successful:"<<T_nowtopre(0,2)<<","<<T_nowtopre(1,2)<<","<<atan2(T_nowtopre(1,0),T_nowtopre(0,0))*57.295<<std::endl;

        Eigen::Matrix3d T_pretoworld;
        T_pretoworld << cos(m_poses_[m_scan_id_-1](2)), -sin(m_poses_[m_scan_id_-1](2)), m_poses_[m_scan_id_-1](0),
                        sin(m_poses_[m_scan_id_-1](2)),  cos(m_poses_[m_scan_id_-1](2)), m_poses_[m_scan_id_-1](1),
                        0, 0, 1;

        Eigen::Matrix3d T_nowtoworld = T_pretoworld*T_nowtopre;
        
        //更新当前位姿
        m_poses_.push_back(Eigen::Vector3d(T_nowtoworld(0, 2),T_nowtoworld(1, 2),atan2(T_nowtoworld(1,0),T_nowtoworld(0,0))));

        //更新局部地图点
        m_local_pts_ = now_local_pts;

        //发布路径
        PubPath(m_poses_[m_scan_id_],m_path_,m_path_pub_);

        //更新全局坐标点
        TOOLS::Translocaltompts(m_local_pts_,m_pts_,m_poses_[m_scan_id_]);

        //建图
        pt_mapper_->Mapping(m_pts_,m_poses_[m_scan_id_]);

        if(m_scan_id_%5==0){
            pt_mapper_->PublishMap(m_map_pub_);
        }

        m_scan_id_++;
    }
    else{
        std::cout <<"IMLS Match Failed!!!!"<<std::endl;
    }

}