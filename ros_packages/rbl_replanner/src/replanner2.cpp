#include "rbl_replanner/replanner.h"

// ===================== CONSTRUCTOR =====================

RBLReplanner::RBLReplanner(const ReplannerParams& params) : params_(params)
{
  std::cout << "[RBLReplanner]: Replanner initialization" << std::endl;

  voxel_size_ = roundToNextMultiple(params.voxel_size, params.replanner_vox_size);
  inflation_  = roundToNextMultiple(params.encumbrance + params.inflation_bonus,
                                   params.replanner_vox_size);

  inflation_coeff_ = std::ceil(inflation_ / params.replanner_vox_size);

  _X_ = std::ceil(params.map_width  / params.replanner_vox_size);
  _Y_ = std::ceil(params.map_length / params.replanner_vox_size);  // FIXED
  _Z_ = std::ceil(params.map_height / params.replanner_vox_size);

  _X_ += _X_ % 2;
  _Y_ += _Y_ % 2;
  _Z_ += _Z_ % 2;

  replanner_period_ = 1.0 / params.replanner_freq;

  first_plan = true;
  goal_changed_ = false;

  _inflated_grid_  = VoxelGrid(_X_, _Y_, _Z_);
  _clearance_grid_ = VoxelGrid(_X_, _Y_, _Z_);
}

// ===================== BASIC SETTERS =====================

void RBLReplanner::setCurrentPosition(const Eigen::Vector3d& point)
{
  agent_pos_ = point;
}

void RBLReplanner::setGoal(const Eigen::Vector3d& point)
{
  if (!goal_.isApprox(point, 1e-6)) {
    goal_changed_ = true;
    goal_ = point;
  }
}

void RBLReplanner::setAltitude(const double& alt)
{
  altitude_ = alt;
}

void RBLReplanner::setPCL(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud)
{
  cloud_ = cloud;
  
}  // //}

// ===================== MAIN PLAN =====================

std::vector<Eigen::Vector3d> RBLReplanner::plan()
{
  initializationPlan();

  fillAndInflateGrid(_inflated_grid_, cloud_);

  _path_ = worldPathToGridPath(path_);

  if (!shouldReplan(path_, agent_pos_, _path_, _inflated_grid_)) {
    return gridPathToWorldPath(smoothPath(_path_, _inflated_grid_));
  }

  calculateClearanceGrid(_clearance_grid_, _inflated_grid_);

  _path_ = AStarPlan(_agent_pos_, _goal_, _path_,
                     _inflated_grid_, _clearance_grid_);

  path_ = gridPathToWorldPath(_path_);

  return gridPathToWorldPath(smoothPath(_path_, _inflated_grid_));
}

// ===================== GRID UTILS =====================

double RBLReplanner::roundToNextMultiple(double value, double multiple)
{
  if (multiple == 0.0) return value;
  return std::ceil(value / multiple) * multiple;
}

// ===================== INFLATION =====================

void RBLReplanner::fillAndInflateGrid(std::optional<VoxelGrid>& grid,
                                      const std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud)
{
  std::vector<std::tuple<int,int,int>> occupied;

  for (const auto& p : cloud->points) {
    auto idx = worldCoordsToGridIdx(p);
    int x,y,z;
    std::tie(x,y,z) = idx;

    if (x>=0 && x<grid->X && y>=0 && y<grid->Y && z>=0 && z<grid->Z) {
      grid->at(x,y,z) = 1;
      occupied.push_back(idx);
    }
  }

  int r = inflation_coeff_;

  for (auto& p : occupied) {
    int x,y,z;
    std::tie(x,y,z) = p;

    for (int dx=-r; dx<=r; dx++)
    for (int dy=-r; dy<=r; dy++)
    for (int dz=-r; dz<=r; dz++) {

      if (dx*dx + dy*dy + dz*dz > r*r) continue;

      int nx = x+dx, ny = y+dy, nz = z+dz;

      if (nx>=0 && nx<grid->X &&
          ny>=0 && ny<grid->Y &&
          nz>=0 && nz<grid->Z)
      {
        grid->at(nx,ny,nz) = 1;
      }
    }
  }

  // floor
  for (int x=0; x<grid->X; x++)
    for (int y=0; y<grid->Y; y++)
      grid->at(x,y,0) = 1;
}

// ===================== DISTANCE TRANSFORM =====================

void RBLReplanner::calculate1dSquaredDistance(int* f, int* d, int n)
{
  std::vector<int> v(n);
  std::vector<double> z(n+1);

  int k = 0;
  v[0] = 0;
  z[0] = -1e20;
  z[1] = 1e20;

  for (int q=1; q<n; q++) {
    double s;
    do {
      int r = v[k];
      s = ((f[q]+q*q)-(f[r]+r*r))/(2.0*(q-r));
      if (s <= z[k]) k--;
      else break;
    } while (k>=0);

    k++;
    v[k]=q;
    z[k]=s;
    z[k+1]=1e20;
  }

  k=0;
  for (int q=0; q<n; q++) {
    while (z[k+1] < q) k++;
    int r = v[k];
    d[q] = (q-r)*(q-r) + f[r];
  }
}

void RBLReplanner::calculateClearanceGrid(std::optional<VoxelGrid>& clearance,
                                          const std::optional<VoxelGrid>& input)
{
  if (!input || !clearance) return;

  const int INF = 1e9;

  for (int x=0;x<_X_;x++)
  for (int y=0;y<_Y_;y++)
  for (int z=0;z<_Z_;z++)
    clearance->at(x,y,z) = input->at(x,y,z) ? 0 : INF;

  std::vector<int> f,d;

  // X
  f.resize(_X_); d.resize(_X_);
  for (int y=0;y<_Y_;y++)
  for (int z=0;z<_Z_;z++) {
    for (int x=0;x<_X_;x++) f[x]=clearance->at(x,y,z);
    calculate1dSquaredDistance(f.data(),d.data(),_X_);
    for (int x=0;x<_X_;x++) clearance->at(x,y,z)=d[x];
  }

  // Y
  f.resize(_Y_); d.resize(_Y_);
  for (int x=0;x<_X_;x++)
  for (int z=0;z<_Z_;z++) {
    for (int y=0;y<_Y_;y++) f[y]=clearance->at(x,y,z);
    calculate1dSquaredDistance(f.data(),d.data(),_Y_);
    for (int y=0;y<_Y_;y++) clearance->at(x,y,z)=d[y];
  }

  // Z
  f.resize(_Z_); d.resize(_Z_);
  for (int x=0;x<_X_;x++)
  for (int y=0;y<_Y_;y++) {
    for (int z=0;z<_Z_;z++) f[z]=clearance->at(x,y,z);
    calculate1dSquaredDistance(f.data(),d.data(),_Z_);
    for (int z=0;z<_Z_;z++)
      clearance->at(x,y,z)=std::sqrt(d[z]);
  }
}

// ===================== SMOOTHING =====================

bool RBLReplanner::canConnectPoints(const std::tuple<int,int,int>& p1,
                                    const std::tuple<int,int,int>& p2,
                                    const std::optional<VoxelGrid>& grid)
{
  int x1,y1,z1,x2,y2,z2;
  std::tie(x1,y1,z1)=p1;
  std::tie(x2,y2,z2)=p2;

  double dist = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)+(z2-z1)*(z2-z1));
  if (dist < 1e-6) return grid->at(x1,y1,z1)==0;

  double step = 0.5;

  for (double t=0; t<=dist; t+=step) {
    double r=t/dist;

    int x=round(x1+(x2-x1)*r);
    int y=round(y1+(y2-y1)*r);
    int z=round(z1+(z2-z1)*r);

    if (x<0||x>=grid->X||y<0||y>=grid->Y||z<0||z>=grid->Z)
      return false;

    if (grid->at(x,y,z)==1)
      return false;
  }

  return true;
}

// ===================== HELPERS =====================

double RBLReplanner::euclideanDistance(const std::tuple<int,int,int>& a,
                                       const std::tuple<int,int,int>& b)
{
  return params_.replanner_vox_size *
    sqrt(pow(std::get<0>(a)-std::get<0>(b),2)+
         pow(std::get<1>(a)-std::get<1>(b),2)+
         pow(std::get<2>(a)-std::get<2>(b),2));
}
