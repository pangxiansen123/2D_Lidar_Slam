#ifndef TOOLS_H
#define TOOLS_H

#include <sensor_msgs/MultiEchoLaserScan.h>
#include <Eigen/Core>
#define GN_PI 3.1415926


namespace TOOLS{

//将激光雷达扫描数据转换为激光坐标系下的局部点云,并去除无效数据
void TransLaserscanTolocalpts(sensor_msgs::MultiEchoLaserScan& scan,std::vector<Eigen::Vector2d>& local_pts){
    local_pts.clear();
    for (int i=0;i<scan.ranges.size();++i){
        //判断扫描数据合法性
        if(scan.ranges[i].echoes[0]<scan.range_min ||
        scan.ranges[i].echoes[0]>scan.range_max ){
            continue;
        }

        double lx = scan.ranges[i].echoes[0] * std::cos(scan.angle_min + i*scan.angle_increment);
        double ly = scan.ranges[i].echoes[0] * -std::sin(scan.angle_min + i*scan.angle_increment);//此处需要将激光雷达的角度取反

        local_pts.push_back(Eigen::Vector2d(lx,ly));
    }
}

//将局部坐标点转换为世界坐标系下的全局坐标点
void Translocaltompts(std::vector<Eigen::Vector2d>& local_pts,std::vector<Eigen::Vector2d>& m_pts,Eigen::Vector3d & pose){
    double tx = pose.x();
    double ty = pose.y();
    double theta = pose.z();

    Eigen::Matrix2d R;
    R << std::cos(theta), -std::sin(theta),
         std::sin(theta),  std::cos(theta);

    m_pts.clear();
    for(int i=0;i<local_pts.size();++i){
        m_pts.push_back(R*local_pts[i] + Eigen::Vector2d(tx,ty));
    }
}

//角度正则化
double NormalizationAngle(double angle)
{
    if(angle > GN_PI)
        angle -= 2*GN_PI;
    else if(angle < -GN_PI)
        angle += 2*GN_PI;

    return angle;
}

}
#endif