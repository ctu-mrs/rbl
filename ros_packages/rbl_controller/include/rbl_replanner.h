#ifndef RBL_REPLANNER_H
#define RBL_REPLANNER_H

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>

#include <iostream>
#include <cmath>
#include <numeric> 
#include <tuple>
#include <vector>
#include <optional>
#include <memory>
#include <chrono>
#include <queue>
#include <set>



struct VoxelGrid {
  int X, Y, Z;
  std::vector<int> data;

  VoxelGrid(int x, int y, int z) : X(x), Y(y), Z(z), data(x * y * z, 0) {}

  int& at(int x, int y, int z) {
    return data[x * Y * Z + y * Z + z];
  }

  // Const version for reading
  const int& at(int x, int y, int z) const {
    return data[x * Y * Z + y * Z + z];
  }

  int& at(std::tuple<int, int, int> pos) {
    return data[std::get<0>(pos) * Y * Z + std::get<1>(pos) * Z + std::get<2>(pos)];
  }

  // Const version for reading
  const int& at(std::tuple<int, int, int> pos) const {
    return data[std::get<0>(pos) * Y * Z + std::get<1>(pos) * Z + std::get<2>(pos)];
  }

  void clear() {
    std::fill(data.begin(), data.end(), 0);
  }

};

struct Node {
  Node* parent;
  std::tuple<int, int, int> position;

  double g;
  double h;
  double f;

  Node(Node* parent = nullptr, std::tuple<int, int, int> position = {0, 0, 0}) {
    this->parent = parent;
    this->position = position;
    this->g = 0;
    this->h = 0;
    this->f = 0;
  }

  bool operator==(const Node& other) const {
    return position == other.position;
  }
};

struct CompareNode {
  bool operator()(const Node* a, const Node* b) const {
      return a->f > b->f;
  }
};

struct ReplannerParams {
  double                                                    encumbrance;
  double                                                    voxel_size;
  double                                                    map_width;
  double                                                    map_height;
  double                                                    weight_safety;
  double                                                    weight_deviation;
  double                                                    replanner_freq;
  double                                                    eps                   = 0.001;
  double                                                    inflation_bonus       = 0.1;
  double                                                    visibility            = 0.1;
  double                                                    replanner_vox_size    = 0.1; 
};

class RBLReplanner {
public:
  RBLReplanner(const ReplannerParams& par);
  void setCurrentPosition(const Eigen::Vector3d& point);
  void setGoal(const Eigen::Vector3d& point);
  void setAltitude(const double& alt);
  void setPCL(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& cloud);


  std::vector<Eigen::Vector3d> getInflatedCloud();

  std::vector<Eigen::Vector3d> plan();
  bool replanTimer();

private:
  ReplannerParams                                           params_;
  double                                                    voxel_size_;
  double                                                    inflation_;
  double                                                    visibility_;
  double                                                    replanner_period_;
  int                                                       inflation_coeff_;
  double                                                    altitude_;
  Eigen::Vector3d                                           agent_pos_;
  Eigen::Vector3d                                           goal_;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>           cloud_;
  std::vector<Eigen::Vector3d>                              path_;
  std::vector<Eigen::Vector3d>                              smooth_path_;
  std::chrono::high_resolution_clock::time_point            last_replan;
  bool                                                      first_plan;
  bool                                                      goal_changed_;
  //Variables related to grid
  int                                                       _X_;
  int                                                       _Y_;
  int                                                       _Z_;
  std::tuple<int, int, int>                                 _agent_pos_;
  std::tuple<int, int, int>                                 _goal_;
  std::vector<std::tuple<int, int, int>>                    _path_;
  std::optional<VoxelGrid>                                  _inflated_grid_;
  std::optional<VoxelGrid>                                  _clearance_grid_;

  bool shouldReplan(const std::vector<Eigen::Vector3d>& path, Eigen::Vector3d& agent_pos, std::vector<std::tuple<int, int, int>> _path, std::optional<VoxelGrid>& grid);
  bool percentageCompleted(const double percentage, const std::vector<Eigen::Vector3d>& path, Eigen::Vector3d& agent_pos);
  bool pathBlocked(std::vector<std::tuple<int, int, int>>& _path, std::optional<VoxelGrid>& grid);

  void initializationPlan();
  double roundToNextMultiple(double value, double multiple);
  Eigen::Vector3d gridIdxToWorldCoords(const std::tuple<int, int, int>& _point);
  Eigen::Vector3d gridIdxToWorldCoords(const int& x, const int& y, const int& z);
  std::tuple<int, int, int> worldCoordsToGridIdx(const Eigen::Vector3d& point);
  std::tuple<int, int, int> worldCoordsToGridIdx(const pcl::PointXYZ& point);
  void fillAndInflateGrid(std::optional<VoxelGrid>& grid, const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& cloud);
  void calculateClearanceGrid(std::optional<VoxelGrid>& clearance_grid, const std::optional<VoxelGrid>& input_grid);
  void calculate1dSquaredDistance(std::vector<int>& data, int length, int stride);
  std::vector<Eigen::Vector3d> gridPathToWorldPath(std::vector<std::tuple<int, int ,int>>& _path);
  std::vector<std::tuple<int, int, int>> worldPathToGridPath(const std::vector<Eigen::Vector3d>& path);
  std::vector<std::tuple<int, int, int>> smoothPath(const std::vector<std::tuple<int, int ,int>>& _path, const std::optional<VoxelGrid>& grid);
  bool canConnectPoints(const std::tuple<int, int, int>& p1, const std::tuple<int, int, int>& p2, const std::optional<VoxelGrid>& grid);
  std::vector<std::tuple<int, int ,int>> AStarPlan(const std::tuple<int, int, int> _start, const std::tuple<int, int, int> _goal, const std::vector<std::tuple<int, int, int>>& _path, const std::optional<VoxelGrid>& grid, const std::optional<VoxelGrid>& clearance_grid);
  double deviationPenalty(const std::vector<std::tuple<int, int, int>>& _path, const std::tuple<int, int, int>& _p1, const std::tuple<int, int, int>& _p2);
  double euclideanDistance(const std::tuple<int, int, int>& p1, const std::tuple<int, int, int>& p2);
  std::tuple<int, int, int> closestFreeIdx(const std::tuple<int, int, int>& _position, const std::optional<VoxelGrid>& grid);
};


#endif
