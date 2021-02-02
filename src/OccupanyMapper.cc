#include "lidar-slam/OccupanyMapper.h"

OccupanyMapper::OccupanyMapper(/* args */)
{
}

OccupanyMapper::~OccupanyMapper()
{
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