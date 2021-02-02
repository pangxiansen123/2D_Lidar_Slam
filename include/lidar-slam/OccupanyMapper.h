#ifndef OCCUPANY_MAPPER_H
#define OCCUPANY_MAPPER_H

#include <iostream>
#include <vector>

#include <ros/ros.h>

#include <Eigen/Core>

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



unsigned char *pMap;

class OccupanyMapper
{
private:
    /* data */
    MapParams mapParams;
public:
    OccupanyMapper(/* args */);
    ~OccupanyMapper();
    std::vector<GridIndex> TraceLine(int x0, int y0, int x1, int y1);
};

#endif