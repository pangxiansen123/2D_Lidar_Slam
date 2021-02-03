#ifndef OCCUPANY_MAPPER_H
#define OCCUPANY_MAPPER_H


#include <iostream>
#include <vector>

#include <ros/ros.h>

#include <Eigen/Core>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>

typedef struct gridindex_
{
    int x;
    int y;

    void SetIndex(int x_, int y_)
    {
        x = x_;
        y = y_;
    }
} GridIndex;

typedef struct map_params
{
    double log_occ, log_free;
    double log_max, log_min;
    double resolution;
    double origin_x, origin_y;
    int height, width;
    int offset_x, offset_y;

} MapParams;


class OccupanyMapper
{
private:
    /* data */
    MapParams m_mapParams_;
    unsigned char *m_pMap_;
public:
    OccupanyMapper(MapParams param);
    ~OccupanyMapper();
    std::vector<GridIndex> TraceLine(int x0, int y0, int x1, int y1);
    GridIndex ConvertWorld2GridIndex(double x, double y);
    int GridIndexToLinearIndex(GridIndex index);
    bool isValidGridIndex(GridIndex index);
    void Mapping(std::vector<Eigen::Vector2d> &m_pts, Eigen::Vector3d& pose);
    void PublishMap(ros::Publisher &map_pub);
};

#endif