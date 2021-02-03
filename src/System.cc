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
    //pt_matcher_ = new Matcher();

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

    m_prevLDP = NULL;
    SetPIICPParams();
}

System::~System()
{

}

void System::SetPIICPParams(){
    
    //设置激光的范围
    m_PIICPParams.min_reading = 0.1;
    m_PIICPParams.max_reading = 60;

    //设置位姿最大的变化范围
    m_PIICPParams.max_angular_correction_deg = 20.0;
    m_PIICPParams.max_linear_correction = 1;

    //设置迭代停止的条件
    m_PIICPParams.max_iterations = 50;
    m_PIICPParams.epsilon_xy = 0.000001;
    m_PIICPParams.epsilon_theta = 0.0000001;

    //设置correspondence相关参数
    m_PIICPParams.max_correspondence_dist = 1;
    m_PIICPParams.sigma = 0.01;
    m_PIICPParams.use_corr_tricks = 1;

    //设置restart过程，因为不需要restart所以可以不管
    m_PIICPParams.restart = 0;
    m_PIICPParams.restart_threshold_mean_error = 0.01;
    m_PIICPParams.restart_dt = 1.0;
    m_PIICPParams.restart_dtheta = 0.1;

    //设置聚类参数
    m_PIICPParams.clustering_threshold = 0.2;

    //用最近的10个点来估计方向
    m_PIICPParams.orientation_neighbourhood = 10;

    //设置使用PI-ICP
    m_PIICPParams.use_point_to_line_distance = 1;

    //不进行alpha_test
    m_PIICPParams.do_alpha_test = 0;
    m_PIICPParams.do_alpha_test_thresholdDeg = 5;

    //设置trimmed参数 用来进行outlier remove
    m_PIICPParams.outliers_maxPerc = 0.9;
    m_PIICPParams.outliers_adaptive_order = 0.7;
    m_PIICPParams.outliers_adaptive_mult = 2.0;

    //进行visibility_test 和 remove double
    m_PIICPParams.do_visibility_test = 1;
    m_PIICPParams.outliers_remove_doubles = 1;
    m_PIICPParams.do_compute_covariance = 0;
    m_PIICPParams.debug_verify_tricks = 0;
    m_PIICPParams.use_ml_weights = 0;
    m_PIICPParams.use_sigma_weights = 0;
}


void System::LaserScanToLDP(sensor_msgs::MultiEchoLaserScanConstPtr& pScan,
                    LDP& ldp)
{
    int nPts = pScan->ranges.size();
    ldp = ld_alloc_new(nPts);

    for(int i = nPts-1;i >= 0 ;--i)
    {
        double dist = pScan->ranges[i].echoes[0];

        if(dist > pScan->range_min && dist < pScan->range_max)
        {
            ldp->valid[nPts-1-i] = 1;
            ldp->readings[nPts-1-i] = dist;
        }
        else
        {
            ldp->valid[nPts-1-i] = 0;
            ldp->readings[nPts-1-i] = -1;
        }
        ldp->theta[nPts-1-i] = -(pScan->angle_min + i* pScan->angle_increment);
    }
    ldp->min_theta = ldp->theta[0];
    ldp->max_theta = ldp->theta[nPts-1];

    ldp->odometry[0] = 0.0;
    ldp->odometry[1] = 0.0;
    ldp->odometry[2] = 0.0;

    ldp->true_pose[0] = 0.0;
    ldp->true_pose[1] = 0.0;
    ldp->true_pose[2] = 0.0;
}

Eigen::Vector3d  System::PIICPBetweenTwoFrames(LDP& currentLDPScan,
                                                  Eigen::Vector3d tmprPose)
{
    m_prevLDP->odometry[0] = 0.0;
    m_prevLDP->odometry[1] = 0.0;
    m_prevLDP->odometry[2] = 0.0;

    m_prevLDP->estimate[0] = 0.0;
    m_prevLDP->estimate[1] = 0.0;
    m_prevLDP->estimate[2] = 0.0;

    m_prevLDP->true_pose[0] = 0.0;
    m_prevLDP->true_pose[1] = 0.0;
    m_prevLDP->true_pose[2] = 0.0;

    //设置匹配的参数值
    m_PIICPParams.laser_ref = m_prevLDP;
    m_PIICPParams.laser_sens = currentLDPScan;

    m_PIICPParams.first_guess[0] = tmprPose(0);
    m_PIICPParams.first_guess[1] = tmprPose(1);
    m_PIICPParams.first_guess[2] = tmprPose(2);

    m_OutputResult.cov_x_m = 0;
    m_OutputResult.dx_dy1_m = 0;
    m_OutputResult.dx_dy2_m = 0;

    sm_icp(&m_PIICPParams,&m_OutputResult);

    //nowPose在lastPose中的坐标
    Eigen::Vector3d  rPose;
    if(m_OutputResult.valid)
    {
        //得到两帧激光之间的相对位姿
        rPose(0)=(m_OutputResult.x[0]);
        rPose(1)=(m_OutputResult.x[1]);
        rPose(2)=(m_OutputResult.x[2]);
    }
    else
    {
        std::cout <<"PI ICP Failed!!!!!!!"<<std::endl;
        rPose = tmprPose;
    }

    return rPose;
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

        //pt_matcher_->setTargetPointCloud(m_local_pts_);

        //更新全局坐标点
        TOOLS::Translocaltompts(m_local_pts_,m_pts_,m_poses_[m_scan_id_]);

        //NDT地图创建
        //m_NDT_matcher_.CreateMapFromLaserPoints(m_poses_[m_scan_id_],m_local_pts_,0.1);

        //更新PLICP参数
        LaserScanToLDP(msg,m_prevLDP);

        //建图
        pt_mapper_->Mapping(m_pts_,m_poses_[m_scan_id_]);

        m_is_first_frame_ = false;
        m_scan_id_++;
        return;
    }


    //将当前扫描数据转换为激光坐标系下的点云
    std::vector<Eigen::Vector2d> now_local_pts;
    TOOLS::TransLaserscanTolocalpts(scan,now_local_pts);

    LDP currentLDP;
    LaserScanToLDP(msg, currentLDP);

    Eigen::Vector3d d_point_scan = PIICPBetweenTwoFrames(currentLDP, Eigen::Vector3d::Zero());

    Eigen::Matrix3d T_nowtopre;
    T_nowtopre << cos(d_point_scan(2)), -sin(d_point_scan(2)), d_point_scan(0),
                sin(d_point_scan(2)),  cos(d_point_scan(2)), d_point_scan(1),
                0,  0,  1;

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

    //更新PLICP参数
    m_prevLDP = currentLDP;

    m_scan_id_++;

    //设置当前帧和前一帧的点云
/*     pt_matcher_->setSourcePointCloud(now_local_pts);

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
    } */
/*     Eigen::Vector3d now_pose;
    if(m_scan_id_ ==1){
        now_pose << 0.0,0.0,0.0;
        m_NDT_matcher_.Optimization(now_pose,now_local_pts);
    }
    else{
        now_pose = m_poses_[m_scan_id_-1] + m_poses_[m_scan_id_-1] - m_poses_[m_scan_id_-2];
        now_pose(2) = TOOLS::NormalizationAngle(now_pose(2));
        m_NDT_matcher_.Optimization(now_pose,now_local_pts);
    }



    m_poses_.push_back(now_pose);
    m_local_pts_ = now_local_pts;

    PubPath(m_poses_[m_scan_id_],m_path_,m_path_pub_);

    TOOLS::Translocaltompts(m_local_pts_,m_pts_,m_poses_[m_scan_id_]);

    pt_mapper_->Mapping(m_pts_,m_poses_[m_scan_id_]);

    if(m_scan_id_%5==0){
        pt_mapper_->PublishMap(m_map_pub_);
    }

    m_NDT_matcher_.map_free();
    m_NDT_matcher_.CreateMapFromLaserPoints(m_poses_[m_scan_id_],m_local_pts_,0.1);

    if(m_scan_id_%5==0){
        pt_mapper_->PublishMap(m_map_pub_);
    }

    m_scan_id_++; */



}