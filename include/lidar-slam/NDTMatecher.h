#ifndef NDT_MATCHER_H
#define NDT_MATHCER_H


#include <iostream>
#include <Eigen/Core>
#include <vector>
#include <queue>
#include <stdlib.h>
#include <stdint.h>
#include <Eigen/Dense>



//地图cell定义
typedef struct
{
  //Occupancy state (-1 = free, 0 = unknown, +1 = occ)
  int occ_state;

  //Distance to the nearest occupied cell
  double occ_dist;

  double score;

} map_cell_t;

//地图定义
typedef struct
{
  // Map origin; the map is a viewport onto a conceptual larger map.
  double origin_x, origin_y;

  // Map scale (m/cell) 地图的分辨率
  double resolution;

  // Map dimensions (number of cells) X Y方向的栅格束
  int size_x, size_y;

  // The map data, stored as a grid
  map_cell_t *cells;

  // Max distance at which we care about obstacles, for constructing
  // likelihood field
  double max_occ_dist; //在似然场模型中，障碍物影响的最大距离

  double min_score;

  double likelihood_sigma; //似然场的标准差

} map_t;

//地图状态相关参数
#define MAP_WIFI_MAX_LEVELS 8
#define CELL_STATUS_FREE    (-1)
#define CELL_STATUS_UNKNOWN (0)
#define CELL_STATUS_OCC     (+1)

//世界坐标转换到地图坐标
#define MAP_GXWX(m_NDT_map_, x) (floor((x - m_NDT_map_->origin_x) / m_NDT_map_->resolution + 0.5) + m_NDT_map_->size_x / 2)
#define MAP_GYWY(m_NDT_map_, y) (floor((y - m_NDT_map_->origin_y) / m_NDT_map_->resolution + 0.5) + m_NDT_map_->size_y / 2)

//把栅格坐标转化为Index
#define MAP_INDEX(m_NDT_map_, i, j) ((i) + (j) * m_NDT_map_->size_x)


class NDTMatecher
{
private:
    /* data */
    map_t * m_NDT_map_;
public:
    NDTMatecher(/* args */);
    ~NDTMatecher();
    void CreateMapFromLaserPoints(Eigen::Vector3d& map_origin_pt,
                                std::vector<Eigen::Vector2d>& laser_pts,
                                double resolution);
    void map_alloc(void);
    void map_update_cspace(double max_occ_dist);
    void Optimization(Eigen::Vector3d& init_pose,std::vector<Eigen::Vector2d>& laser_pts);
    void ComputeHessianAndb(Eigen::Vector3d now_pose,std::vector<Eigen::Vector2d>& laser_pts,Eigen::Matrix3d& H, Eigen::Vector3d& b);
    Eigen::Vector3d InterpMapValueWithDerivatives(Eigen::Vector2d& coords);
    void map_free();
};




#endif