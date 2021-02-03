#include "lidar-slam/NDTMatecher.h"

class CachedDistanceMap
{
  public:
    CachedDistanceMap(double scale, double max_dist) :
      distances_(NULL), scale_(scale), max_dist_(max_dist)
    {
      //最大距离对应的cell的个数
      cell_radius_ = max_dist / scale;

      distances_ = new double *[cell_radius_+2];
      for(int i=0; i<=cell_radius_+1; i++)
      {
        distances_[i] = new double[cell_radius_+2];
        for(int j=0; j<=cell_radius_+1; j++)
        {
            distances_[i][j] = sqrt(i*i + j*j);
        }
      }
    }

    ~CachedDistanceMap()
    {
      if(distances_)
      {
        for(int i=0; i<=cell_radius_+1; i++)
            delete[] distances_[i];
        delete[] distances_;
      }
    }
    double** distances_;
    double scale_;
    double max_dist_;
    int cell_radius_;
};


class CellData
{
  public:
    map_t* map_;
    unsigned int i_, j_;
    unsigned int src_i_, src_j_;
};
bool operator<(const CellData& a, const CellData& b)
{
  return a.map_->cells[MAP_INDEX(a.map_, a.i_, a.j_)].occ_dist > a.map_->cells[MAP_INDEX(b.map_, b.i_, b.j_)].occ_dist;
}


//将位姿向量转换为变换矩阵
void TransvtoT(Eigen::Vector3d& pose,Eigen::Matrix3d& T){

    T   <<  std::cos(pose(2)),-std::sin(pose(2)),pose(0),
            std::sin(pose(2)), std::cos(pose(2)),pose(1),
            0,           0,     1;
}

//对某一个点进行转换
void Transpoint(Eigen::Vector2d&pt ,Eigen::Matrix3d& T , Eigen::Vector2d& target_pt){

    Eigen::Vector3d tmp_pt(pt(0),pt(1),1);
    tmp_pt = T * tmp_pt;
    target_pt << tmp_pt(0),tmp_pt(1);
}

NDTMatecher::NDTMatecher(/* args */)
{
    m_NDT_map_ = NULL;
}

NDTMatecher::~NDTMatecher()
{
    if(m_NDT_map_!= NULL){
        delete m_NDT_map_;
    }
}

//创建新地图
void NDTMatecher::map_alloc(void)
{
    //删除前一次的地图
    if(m_NDT_map_ != NULL){
        delete m_NDT_map_;
        m_NDT_map_ = NULL;
    }

    m_NDT_map_ = (map_t*) malloc(sizeof(map_t));

    // Assume we start at (0, 0)
    m_NDT_map_->origin_x = 0;
    m_NDT_map_->origin_y = 0;

    // Make the size odd
    m_NDT_map_->size_x = 0;
    m_NDT_map_->size_y = 0;
    m_NDT_map_->resolution = 0;

    // Allocate storage for main map
    m_NDT_map_->cells = (map_cell_t*) NULL;
  
}


CachedDistanceMap* get_distance_map(double scale, double max_dist)
{
  static CachedDistanceMap* cdm = NULL;

  if(!cdm || (cdm->scale_ != scale) || (cdm->max_dist_ != max_dist))
  {
    if(cdm)
      delete cdm;
    cdm = new CachedDistanceMap(scale, max_dist);
  }

  return cdm;
}

//队列扩展
void enqueue(map_t* m_NDT_map_,unsigned int i, unsigned int j,
	     unsigned int src_i, unsigned int src_j,
	     std::priority_queue<CellData>& Q,
	     CachedDistanceMap* cdm,
	     unsigned char* marked)
{
  //如果已经被计算过了 则直接返回
  if(marked[MAP_INDEX(m_NDT_map_, i, j)])
    return;
  


  //这里的距离是栅格的距离
  unsigned int di = abs((int)i - (int)src_i);
  unsigned int dj = abs((int)j - (int)src_j);
  double distance = cdm->distances_[di][dj];

  if(distance > cdm->cell_radius_)
    return;

  //转换为实际距离
  m_NDT_map_->cells[MAP_INDEX(m_NDT_map_, i, j)].occ_dist = distance * m_NDT_map_->resolution;

  double z = m_NDT_map_->cells[MAP_INDEX(m_NDT_map_,i,j)].occ_dist;
  m_NDT_map_->cells[MAP_INDEX(m_NDT_map_, i, j)].score = exp(-(z * z) /  (2 * m_NDT_map_->likelihood_sigma * m_NDT_map_->likelihood_sigma));

  CellData cell;
  cell.map_ = m_NDT_map_;
  cell.i_ = i;
  cell.j_ = j;
  cell.src_i_ = src_i;
  cell.src_j_ = src_j;

  Q.push(cell);

  marked[MAP_INDEX(m_NDT_map_, i, j)] = 1;
}


//创建似然场
void NDTMatecher::map_update_cspace(double max_occ_dist)
{
  unsigned char* marked;
  std::priority_queue<CellData> Q;

  marked = new unsigned char[m_NDT_map_->size_x*m_NDT_map_->size_y];
  memset(marked, 0, sizeof(unsigned char) * m_NDT_map_->size_x*m_NDT_map_->size_y);

  m_NDT_map_->max_occ_dist = max_occ_dist;

  //得到一个CachedDistanceMap
  CachedDistanceMap* cdm = get_distance_map(m_NDT_map_->resolution, m_NDT_map_->max_occ_dist);

  //这个sigma已经在外面设置过了 在handmapmsg里面就会设置
  m_NDT_map_->min_score = exp(-max_occ_dist * max_occ_dist / (2 * m_NDT_map_->likelihood_sigma * m_NDT_map_->likelihood_sigma));

  // 所有的障碍物都放入队列中
  CellData cell;
  cell.map_ = m_NDT_map_;

  //计算出来所有的边界障碍物 只有边界障碍物才用来进行匹配 其他的障碍物都当成no-information

    /*所有障碍物的栅格  离障碍物的距离都标志为0  非障碍物的栅格都标记为max_occ_dist*/
    for(int i=0; i<m_NDT_map_->size_x; i++)
    {
        cell.src_i_ = cell.i_ = i;
        for(int j=0; j<m_NDT_map_->size_y; j++)
        {
            if(m_NDT_map_->cells[MAP_INDEX(m_NDT_map_, i, j)].occ_state == CELL_STATUS_OCC)
            {
                m_NDT_map_->cells[MAP_INDEX(m_NDT_map_, i, j)].occ_dist = 0.0;
                m_NDT_map_->cells[MAP_INDEX(m_NDT_map_,i,j)].score = 1.0;
                cell.src_j_ = cell.j_ = j;
                marked[MAP_INDEX(m_NDT_map_, i, j)] = 1;
                Q.push(cell);
            }
            else
                m_NDT_map_->cells[MAP_INDEX(m_NDT_map_, i, j)].occ_dist = max_occ_dist;
        }
    }

  while(!Q.empty())
  {
    CellData current_cell = Q.top();

    /*往上、下、左、右四个方向拓展*/
    if(current_cell.i_ > 0)
      enqueue(m_NDT_map_,current_cell.i_-1, current_cell.j_,
	      current_cell.src_i_, current_cell.src_j_,
	      Q, cdm, marked);

    if(current_cell.j_ > 0)
      enqueue(m_NDT_map_,current_cell.i_, current_cell.j_-1,
	      current_cell.src_i_, current_cell.src_j_,
	      Q, cdm, marked);

    if((int)current_cell.i_ <  m_NDT_map_->size_x - 1)
      enqueue(m_NDT_map_,current_cell.i_+1, current_cell.j_,
	      current_cell.src_i_, current_cell.src_j_,
	      Q, cdm, marked);

    if((int)current_cell.j_ <  m_NDT_map_->size_y - 1)
      enqueue(m_NDT_map_,current_cell.i_, current_cell.j_+1,
	      current_cell.src_i_, current_cell.src_j_,
	      Q, cdm, marked);

    Q.pop();
  }

  delete[] marked;
}

//从地图点构建似然场地图
void NDTMatecher::CreateMapFromLaserPoints(Eigen::Vector3d& map_origin_pt,std::vector<Eigen::Vector2d>& laser_pts,double resolution){

    //! 第一步 初始化地图指针
    map_alloc();

    //! 第二步，根据传入参数为地图赋值
    m_NDT_map_->origin_x = map_origin_pt.x();
    m_NDT_map_->origin_y = map_origin_pt.y();
    m_NDT_map_->resolution = resolution;
    //固定大小的地图，必要时可以扩大
    m_NDT_map_->size_x = 10000;
    m_NDT_map_->size_y = 10000;
    //为每一个栅格分配内存
    m_NDT_map_->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*m_NDT_map_->size_x*m_NDT_map_->size_y);

    //高斯平滑的sigma－－固定死
    m_NDT_map_->likelihood_sigma = 0.5;

    //将位姿转换为变换矩阵
    Eigen::Matrix3d Trans;
    TransvtoT(map_origin_pt,Trans);

    //! 第三步，设置障碍物
    for(int i = 0; i < laser_pts.size();i++)
    {
        Eigen::Vector2d tmp_pt;
        Transpoint(laser_pts[i],Trans,tmp_pt);

        //将地图坐标系转换为栅格坐标系
        int cell_x,cell_y;
        cell_x = MAP_GXWX(m_NDT_map_,tmp_pt(0));
        cell_y = MAP_GYWY(m_NDT_map_,tmp_pt(1));

        m_NDT_map_->cells[MAP_INDEX(m_NDT_map_,cell_x,cell_y)].occ_state = CELL_STATUS_OCC;
    }

    //! 第四步，进行障碍物的膨胀--最大距离固定死
    map_update_cspace(0.5);
}

Eigen::Vector3d NDTMatecher::InterpMapValueWithDerivatives(Eigen::Vector2d& coords)
{
    Eigen::Vector3d ans;
    //TODO
    //将点坐标转换到地图坐标系中的坐标索引
    double index_x = (coords(0) - m_NDT_map_->origin_x) / m_NDT_map_->resolution + m_NDT_map_->size_x / 2;
    double index_y = (coords(1) - m_NDT_map_->origin_y) / m_NDT_map_->resolution + m_NDT_map_->size_y / 2;
    int16_t index_x0 = floor(index_x);
    int16_t index_y0 = floor(index_y);
    
    //!SECTION - 注意此处周围四个点之间的距离都是单位1
    double u, v;
    u = index_x - index_x0;
    v = index_y - index_y0;
    
    double z1 = m_NDT_map_->cells[MAP_INDEX(m_NDT_map_, index_x0, index_y0)].score;
    double z2 = m_NDT_map_->cells[MAP_INDEX(m_NDT_map_, index_x0 + 1, index_y0)].score;
    double z3 = m_NDT_map_->cells[MAP_INDEX(m_NDT_map_, index_x0 + 1, index_y0 + 1)].score;
    double z4 = m_NDT_map_->cells[MAP_INDEX(m_NDT_map_, index_x0, index_y0 + 1)].score;
    
    //获取分值
    ans(0) = (1 - u) * (1 - v) * z1 + u * (1 - v) * z2 + u * v * z3 + (1 - u) * v *z4;
    
    //计算梯度
    ans(1) = (v * (z3 - z4) + (1 - v) * (z2 - z1)) / m_NDT_map_->resolution;
    ans(2) = (u * (z3 - z2) + (1 - u) * (z4 - z1)) / m_NDT_map_->resolution;
    //TODO

    return ans;
}

void NDTMatecher::ComputeHessianAndb(Eigen::Vector3d now_pose,
                        std::vector<Eigen::Vector2d>& laser_pts,
                        Eigen::Matrix3d& H, Eigen::Vector3d& b)
{
    H = Eigen::Matrix3d::Zero();
    b = Eigen::Vector3d::Zero();

    //TODO
    //遍历每一个点
    for (Eigen::Vector2d pt: laser_pts) {

        //计算激光点在全局坐标系下的位姿
        Eigen::Matrix3d T = Eigen::Matrix3d::Zero();
        T << cos(now_pose(2)), -sin(now_pose(2)), now_pose(0),
        sin(now_pose(2)),cos(now_pose(2)), now_pose(1),
        0,0,1;
        
        Eigen::Vector2d pt_pose;
        Transpoint(pt,T,pt_pose);

        //计算坐标对于位姿的导数
        Eigen::Matrix<double, 2, 3> ds;
        ds << 1, 0, -sin(now_pose(2) * pt(0)) - cos(now_pose(2)) * pt(1),
                0, 1, cos(now_pose(2) * pt(0)) - sin(now_pose(2)) * pt(1);

        //计算似然场对于坐标位置的导数
        Eigen::Vector3d score_gradient = InterpMapValueWithDerivatives(pt_pose);
        Eigen::Vector2d gradient(score_gradient(1), score_gradient(2));
        double score = score_gradient(0);

        /// noticed the dimension of J should 1 x 3
        Eigen::RowVector3d J = gradient.transpose() * ds;
        H += J.transpose() * J;
        b += J.transpose() * (1 - score);

    }
    //END OF TODO
}

//位姿优化
void NDTMatecher::Optimization(Eigen::Vector3d& init_pose,std::vector<Eigen::Vector2d>& laser_pts)
{
    //最大迭代次数
    int maxIteration = 20;
    Eigen::Vector3d now_pose = init_pose;

    //构造H和b矩阵
    Eigen::Matrix3d H;
    Eigen::Vector3d b;
    for(int i = 0; i < maxIteration;i++)
    {
        //TODO
        ComputeHessianAndb(now_pose, laser_pts, H, b);
        Eigen::Vector3d delta_x = H.colPivHouseholderQr().solve(b);
        now_pose += delta_x;
        //TODO
    }
    init_pose = now_pose;
}

void NDTMatecher::map_free()
{
  free(m_NDT_map_->cells);
  free(m_NDT_map_);
  m_NDT_map_ = NULL;
  return;
}