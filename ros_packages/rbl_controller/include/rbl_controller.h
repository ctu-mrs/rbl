#ifndef RBL_CONTROLLER_H
#define RBL_CONTROLLER_H

#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Eigen>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <mrs_msgs/Reference.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <mrs_msgs/Vec4.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/scope_timer.h>
#include "rbl_controller/ActivateParams.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <filesystem>
#include <boost/make_shared.hpp>
#include <queue>
#include <unordered_map>
#include <tuple>
#include <string>
#include <vector>
#include <set>
#include <iostream>
#include <cmath>
#include <limits>
#include <algorithm>
#include <random>
#include <deque>
#include <utility>
#include <chrono>
#include <optional>
#include <mutex>
#include <future>
#include "rbl_replanner.h"
#include "ciri.h"

struct RBLParams {
  double                                step_size;
  double                                radius;
  double                                encumbrance;
  double                                dt;
  double                                beta_min;
  double                                betaD;
  double                                d1;
  double                                d2;
  double                                d3;
  double                                d4;
  double                                d5;
  double                                d6;
  double                                d7;
  double                                cwvd_rob;
  double                                cwvd_obs;
  bool                                  use_z_rule;
  double                                z_min;
  double                                z_max;
  double                                boundary_threshold;
  double                                boundary_threshold_speed;
  bool                                  use_garmin_alt                = false;
  bool                                  only_2d                       = false;
  double                                z_ref                         = 1.0;
  bool                                  use_map                       = false;
  double                                voxel_size                    = 0.4; //discretization for faster processing of obstacles for raw pcl and also needs to be set if map is used
  bool                                  replanner                     = false; //if true the alg also needs garmin alt - ground truth. For replanner map does not map bellow uav at the start;
  bool                                  limited_fov                   = true;
  bool                                  ciri                          = false;
};

class RBLController {
public:
  RBLController(const RBLParams& par);
  void setCurrentPosition(const Eigen::Vector3d& point);
  void setCurrentVelocity(const Eigen::Vector3d& point);
  void setGroupPositions(const std::vector<Eigen::Vector3d>& list_points);
  void setPCL(const sensor_msgs::PointCloud2::ConstPtr& list_points);
  void setGoal(const Eigen::Vector3d& point);
  void setAltitude(const double& alt);
  void setRollPitchYaw(const Eigen::Vector3d& rpy);

  std::optional<mrs_msgs::Reference>            getNextRef();
  Eigen::Vector3d                               getGoal(); 
  Eigen::Vector3d                               getWaypoint();
  Eigen::Vector3d                               getCurrentPosition();
  Eigen::Vector3d                               getCurrentVelocity();
  Eigen::Vector3d                               getCentroid();
  std::vector<Eigen::Vector3d>                  getCellA();
  std::vector<Eigen::Vector3d>                  getInflatedMap();
  std::vector<Eigen::Vector3d>                  getPath();
  pcl::PointCloud<pcl::PointXYZ>                getPCL();

private:
  RBLParams                                                 params_;

  double                                                    radius_sensing_;
  double                                                    altitude_;
  double                                                    beta_;
  double                                                    ph_; //vertical
  double                                                    th_; //azimuthal
  Eigen::Vector3d                                           goal_; //final goal where the uav will converge
  Eigen::Vector3d                                           destination_; //rotated current goal/waypoint
  Eigen::Vector3d                                           waypoint_; //replanner waypoint
  Eigen::Vector3d                                           agent_pos_; 
  Eigen::Vector3d                                           agent_vel_; 
  Eigen::Vector3d                                           rpy_; 
  Eigen::Vector3d                                           c1_;
  Eigen::Vector3d                                           c2_;
  Eigen::Vector3d                                           c1_no_rot_;
  std::vector<Eigen::Vector3d>                              neighbors_pos_;
  std::vector<Eigen::Vector3d>                              cell_A_;
  std::vector<Eigen::Vector3d>                              cell_S_;
  std::vector<Eigen::Vector3d>                              plane_normals_;
  std::vector<Eigen::Vector3d>                              plane_points_;
  std::vector<Eigen::Vector3d>                              inflated_map_;
  std::vector<Eigen::Vector3d>                              path_;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>           cloud_;
  std::shared_ptr<RBLReplanner>                             rbl_replanner_;
  std::shared_ptr<CIRI>                                     ciri_solver_;
  std::future<std::vector<Eigen::Vector3d>>                 replanner_future_;
  std::mutex                                                replanner_mutex_;

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getGroundCleanCloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& cloud, const Eigen::Vector3d& agent_pos, const double& altitude);
  void voxelizePcl(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& cloud, double voxel_size);
  std::vector<Eigen::Vector3d> getpointsInsideCircle(const Eigen::Vector3d& center, const double& radius, const double& step_size);
  void pointsInsideSphere(std::vector<Eigen::Vector3d>& sphere, const Eigen::Vector3d& center, const double& radius, const double& step_size, const double& altitude);
  void partitionCellA(std::vector<Eigen::Vector3d>&                             cell_A, 
                      std::vector<Eigen::Vector3d>&                             cell_S, 
                      std::vector<Eigen::Vector3d>&                             plane_normals,
                      std::vector<Eigen::Vector3d>&                             plane_points,
                      const Eigen::Vector3d&                                    agent_pos,
                      const std::vector<Eigen::Vector3d>&                       neighbors,
                      std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>&          cloud);
  bool partitionCellACiri(std::vector<Eigen::Vector3d>&                    cell_A,
                          std::vector<Eigen::Vector3d>&                    cell_S,
                          std::vector<Eigen::Vector3d>&                    plane_normals,
                          std::vector<Eigen::Vector3d>&                    plane_points,
                          const Eigen::Vector3d&                           agent_pos,
                          const Eigen::Vector3d&                           waypoint,
                          const std::vector<Eigen::Vector3d>&              neighbors,
                          std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& cloud,
                          const Eigen::Vector3d&                           c1);
  void convertPlaneData(const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& plane_data, std::vector<Eigen::Vector3d>& plane_normals, std::vector<Eigen::Vector3d>& plane_points, const Eigen::Vector3d& agent_pos);
  void closestPointOnVoxel(Eigen::Vector3d& point, const Eigen::Vector3d& agent_pos, const Eigen::Vector3d& voxel_center, const double& voxel_size);
  void createAndPartitionCellA(std::vector<Eigen::Vector3d>& cell_A, std::vector<Eigen::Vector3d>& cell_S, std::vector<Eigen::Vector3d>& plane_normals, std::vector<Eigen::Vector3d>& plane_points, const Eigen::Vector3d& agent_pos, const Eigen::Vector3d& waypoint, const std::vector<Eigen::Vector3d>& neighbors_pos, std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& cloud, const double& altitude, const Eigen::Vector3d& c1);
  void computeCentroid(Eigen::Vector3d& centroid, Eigen::Vector3d& agent_pos, Eigen::Vector3d& agent_vel, std::vector<Eigen::Vector3d>& cell, std::vector<Eigen::Vector3d>& plane_normals, std::vector<Eigen::Vector3d>& plane_points, Eigen::Vector3d& destination, double& beta);
  void computeScalarValue(std::vector<double>& scalar_values, const std::vector<double>& x_test, const std::vector<double>& y_test, const std::vector<double>& z_test, const Eigen::Vector3d &destination, double beta);
  void applyRules(double& beta, double& th, double& ph, Eigen::Vector3d destination, 
                  const Eigen::Vector3d goal, const Eigen::Vector3d& agent_pos, const Eigen::Vector3d& c1, const Eigen::Vector3d& c2, const Eigen::Vector3d& c1_no_rot,
                  const double& d1, const double& d2, const double& d3, const double& d4, const double& d5, const double& d6, const double& d7, const double& betaD, const double& beta_min, const double& dt);
  Eigen::Vector3d determineWaypoint(const std::vector<Eigen::Vector3d>& path, const Eigen::Vector3d& agent_pos, const Eigen::Vector3d& goal);
  void determineNextRef(mrs_msgs::Reference& p_ref, const Eigen::Vector3d& agent_pos, const Eigen::Vector3d& waypoint, const Eigen::Vector3d& goal, const Eigen::Vector3d& c1, const Eigen::Vector3d& rpy, const std::vector<Eigen::Vector3d>& path);
  mrs_msgs::Reference pRefAgent(const Eigen::Vector3d& agent_pos, const double yaw);
  double determineYaw(const Eigen::Vector3d& agent_pos, const Eigen::Vector3d& waypoint, const std::vector<Eigen::Vector3d>& path, const Eigen::Vector3d& rpy);
  // double determineYaw(const Eigen::Vector3d& agent_pos, const std::vector<Eigen::Vector3d>& path, const Eigen::Vector3d& rpy);
  double normalizeAngle(double angle);
};

#endif
