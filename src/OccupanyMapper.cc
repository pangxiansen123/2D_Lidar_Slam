#include "lidar-slam/OccupanyMapper.h"

//初始化
OccupanyMapper::OccupanyMapper(MapParams param)
{
    m_mapParams_ = param;

    m_pMap_ = new unsigned char[m_mapParams_.width * m_mapParams_.height];
    for (int i = 0; i < m_mapParams_.width * m_mapParams_.height; i++)
    {
        m_pMap_[i] = 50;
    }
}

OccupanyMapper::~OccupanyMapper()
{
    delete m_pMap_;
}

//返回从起点到终点一条直线上的所有栅格
std::vector<GridIndex> OccupanyMapper::TraceLine(int x0, int y0, int x1, int y1)
{
    GridIndex tmpIndex;
    std::vector<GridIndex> gridIndexVector;

    bool steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep)
    {
        std::swap(x0, y0);
        std::swap(x1, y1);
    }
    if (x0 > x1)
    {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    int deltaX = x1 - x0;
    int deltaY = abs(y1 - y0);
    int error = 0;
    int ystep;
    int y = y0;

    if (y0 < y1)
    {
        ystep = 1;
    }
    else
    {
        ystep = -1;
    }

    int pointX;
    int pointY;
    for (int x = x0; x <= x1; x++)
    {
        if (steep)
        {
            pointX = y;
            pointY = x;
        }
        else
        {
            pointX = x;
            pointY = y;
        }

        error += deltaY;

        if (2 * error >= deltaX)
        {
            y += ystep;
            error -= deltaX;
        }

        //不包含最后一个点．
        if (pointX == x1 && pointY == y1)
            continue;

        //保存所有的点
        tmpIndex.SetIndex(pointX, pointY);

        gridIndexVector.push_back(tmpIndex);
    }

    return gridIndexVector;
}

//从世界坐标系转换到栅格坐标系
GridIndex OccupanyMapper::ConvertWorld2GridIndex(double x, double y)
{
    GridIndex index;

    index.x = std::ceil((x - m_mapParams_.origin_x) / m_mapParams_.resolution) + m_mapParams_.offset_x;
    index.y = std::ceil((y - m_mapParams_.origin_y) / m_mapParams_.resolution) + m_mapParams_.offset_y;

    return index;
}

//从栅格坐标转换为线性坐标
int OccupanyMapper::GridIndexToLinearIndex(GridIndex index)
{
    int linear_index;
    linear_index = index.y + index.x * m_mapParams_.width;
    return linear_index;
}

//判断栅格坐标是否有效
bool OccupanyMapper::isValidGridIndex(GridIndex index)
{
    if (index.x >= 0 && index.x < m_mapParams_.width && index.y >= 0 && index.y < m_mapParams_.height)
        return true;

    return false;
}

//建图
void OccupanyMapper::Mapping(std::vector<Eigen::Vector2d> &m_pts, Eigen::Vector3d& pose)
{
    GridIndex robotIndex = ConvertWorld2GridIndex(pose.x(),pose.y());
    for (int i=0;i<m_pts.size();++i){

        double world_x = m_pts[i].x();
        double world_y = m_pts[i].y();

        //把激光击中位置转换到地图坐标系
        GridIndex mapIndex = ConvertWorld2GridIndex(world_x,world_y);
        if(!isValidGridIndex(mapIndex)) continue;
        //获取从当前位置到激光击中位置的所有点
        std::vector<GridIndex> freeIndex = TraceLine(robotIndex.x,robotIndex.y,mapIndex.x,mapIndex.y);
        //更新每一个free点
        for (int i=0;i<freeIndex.size();i++){
             //获取地图栅格点索引
             int map_Index = GridIndexToLinearIndex(freeIndex[i]);
             if((double)(m_pMap_[map_Index]+m_mapParams_.log_free)<m_mapParams_.log_min){
                m_pMap_[map_Index] = (m_mapParams_.log_min);
            }
             else{
                m_pMap_[map_Index]+= m_mapParams_.log_free;
            }
        }
        //更新占据点
        if(((double)m_pMap_[GridIndexToLinearIndex(mapIndex)]+m_mapParams_.log_occ)>m_mapParams_.log_max){
            m_pMap_[GridIndexToLinearIndex(mapIndex)] = m_mapParams_.log_max;
        }else{
            m_pMap_[GridIndexToLinearIndex(mapIndex)] += m_mapParams_.log_occ;
        }
    }
}

//发布地图
void OccupanyMapper::PublishMap(ros::Publisher &map_pub){
    nav_msgs::OccupancyGrid rosMap;

    rosMap.info.resolution = m_mapParams_.resolution;


    rosMap.info.origin.position.x = -m_mapParams_.width*m_mapParams_.resolution/2;
    rosMap.info.origin.position.y = -m_mapParams_.height*m_mapParams_.resolution/2;
    rosMap.info.origin.position.z = 0.0;

    tf::Quaternion q;
    q.setRPY(3.14,0.0,1.57);
    rosMap.info.origin.orientation.x = q.x();
    rosMap.info.origin.orientation.y = q.y();
    rosMap.info.origin.orientation.z = q.z();
    rosMap.info.origin.orientation.w = q.w();

    rosMap.info.width = m_mapParams_.width;
    rosMap.info.height = m_mapParams_.height;
    rosMap.data.resize(rosMap.info.width * rosMap.info.height);

    for (int i = 0; i < m_mapParams_.width * m_mapParams_.height; i++)
    {
        if (m_pMap_[i] == 50)
        {
            rosMap.data[i] = -1.0;
        }
        else
        {

            rosMap.data[i] = (int8_t)m_pMap_[i];
        }
    }

    rosMap.header.stamp = ros::Time::now();
    rosMap.header.frame_id = "odom";

    map_pub.publish(rosMap);
}