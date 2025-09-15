#include "rbl_controller.h"

RBLController::RBLController(const RBLParams& params) : params_(params)
{
  radius_sensing_ = params_.radius / params_.cwvd_obs + sqrt((params_.voxel_size / 2) * (params_.voxel_size / 2));
  beta_ = params_.beta_min;
  if (params.replanner) {
    ReplannerParams replanner_params;
    replanner_params.encumbrance = params.encumbrance;
    replanner_params.voxel_size = params.voxel_size;
    replanner_params.map_width = 20.0;
    replanner_params.map_height = 10.0;
    replanner_params.weight_safety = 1.0;
    replanner_params.weight_deviation = 100.0;
    replanner_params.inflation_bonus = 0.0;
    replanner_params.replanner_vox_size = 0.2;
    replanner_params.replanner_freq = 1.0; //[Hz]

    rbl_replanner_ = std::make_shared<RBLReplanner>(replanner_params);
  }

  if (params.ciri) {
    ciriParams  ciri_params;
    ciri_params.epsilon = 1e-6;
    ciri_params.inflation = params.encumbrance + params.voxel_size/2;
    ciri_solver_ = std::make_shared<CIRI>(ciri_params);
  }
}

void RBLController::setCurrentPosition(const Eigen::Vector3d& point)
{
  agent_pos_ = point;
  if (!params_.use_garmin_alt) {
    altitude_ = agent_pos_.z();
  }
}

void RBLController::setGroupPositions(const std::vector<Eigen::Vector3d>& list_points)
{
  neighbors_pos_ = list_points;
}

void RBLController::setPCL(const sensor_msgs::PointCloud2::ConstPtr& list_points)
{
  if (list_points) {
    pcl::PointCloud<pcl::PointXYZ> temp_cloud;
    pcl::fromROSMsg(*list_points, temp_cloud);
    cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(temp_cloud);
  }
}

void RBLController::setGoal(const Eigen::Vector3d& point)
{
  goal_        = point;
  destination_ = point;
  ph_          = 0.0;
  th_          = 0.0;
}

void RBLController::setAltitude(const double& alt)
{
  altitude_ = alt;
}

void RBLController::setRollPitchYaw(const Eigen::Vector3d& rpy)
{
  rpy_ = rpy;
}

std::optional<mrs_msgs::Reference> RBLController::getNextRef()
{
  mrs_msgs::Reference p_ref;

  auto no_ground_cloud = getGroundCleanCloud(cloud_, agent_pos_, altitude_);

  if (!no_ground_cloud) {
    return std::nullopt;
  }

  if (params_.replanner) {
    if (rbl_replanner_->replanTimer()) {
      rbl_replanner_->setAltitude(altitude_);
      rbl_replanner_->setCurrentPosition(agent_pos_);
      rbl_replanner_->setGoal(goal_);
      rbl_replanner_->setPCL(cloud_);
      path_ = rbl_replanner_->plan();
      inflated_map_ = rbl_replanner_->getInflatedCloud();
    }
    waypoint_ = determineWaypoint(path_, agent_pos_, goal_);
    destination_ = waypoint_;
  }

  cell_A_.clear();
  cell_S_.clear();
  createAndPartitionCellA(cell_A_, cell_S_, agent_pos_, waypoint_, neighbors_pos_, no_ground_cloud, altitude_);

  if (params_.replanner) {
    computeCentroid(c1_, cell_A_, destination_, beta_);
    computeCentroid(c2_, cell_S_, destination_, beta_);
    computeCentroid(c1_no_rot_, cell_A_, waypoint_, beta_);
    applyRules(beta_, th_, ph_, destination_, waypoint_, agent_pos_, c1_, c2_, c1_no_rot_, params_.d1, params_.d2, params_.d3, params_.d4, params_.d5, params_.d6, params_.d7, params_.betaD, params_.beta_min, params_.dt);
  } else {
    computeCentroid(c1_, cell_A_, destination_, beta_);
    computeCentroid(c2_, cell_S_, destination_, beta_);
    computeCentroid(c1_no_rot_, cell_A_, goal_, beta_);
    applyRules(beta_, th_, ph_, destination_, goal_, agent_pos_, c1_, c2_, c1_no_rot_, params_.d1, params_.d2, params_.d3, params_.d4, params_.d5, params_.d6, params_.d7, params_.betaD, params_.beta_min, params_.dt);
  }
  
  // std::cout << "[RBLController]: Altitude: " << altitude_ << std::endl;
  // std::cout << "[RBLController]: Agent posisiton  x: " << agent_pos_.x() << ", y: " << agent_pos_.y() << ", z: " << agent_pos_.z() << std::endl; 
  // std::cout << "[RBLController]: Destination  x: " << destination_.x() << ", y: " << destination_.y() << ", z: " << destination_.z() << std::endl; 
  // std::cout << "[RBLController]: Waypoint  x: " << waypoint_.x() << ", y: " << waypoint_.y() << ", z: " << waypoint_.z() << std::endl; 
  // std::cout << "[RBLController]: Goal  x: " << goal_.x() << ", y: " << goal_.y() << ", z: " << goal_.z() << std::endl; 
  // std::cout << "[RBLController]: Centroid c1 x: " << c1_.x() << ", y: " << c1_.y() << ", z: " << c1_.z() << std::endl;

  // applyRules(beta_,
  //            th_,
  //            ph_,
  //            destination_,
  //            goal_,
  //            agent_pos_,
  //            c1_,
  //            c2_,
  //            c1_no_rot_,
  //            params_.d1,
  //            params_.d2,
  //            params_.d3,
  //            params_.d4,
  //            params_.d5,
  //            params_.d6,
  //            params_.d7,
  //            params_.betaD,
  //            params_.beta_min,
  //            params_.dt);

  determineNextRef(p_ref, agent_pos_, goal_, c1_, rpy_, path_);
  
  return p_ref;
}

Eigen::Vector3d RBLController::getGoal()
{
  return goal_;
}

Eigen::Vector3d RBLController::getWaypoint()
{
  return waypoint_;
}

Eigen::Vector3d RBLController::getCurrentPosition()
{
  return agent_pos_;
}

Eigen::Vector3d RBLController::getCentroid()
{
  return c1_;
}

std::vector<Eigen::Vector3d> RBLController::getCellA()
{
  return cell_A_;
}

std::vector<Eigen::Vector3d> RBLController::getInflatedMap()
{
  return inflated_map_;
}

std::vector<Eigen::Vector3d> RBLController::getPath()
{
  if (params_.replanner) {
    return path_;
  }
  return {};
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> RBLController::getGroundCleanCloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& cloud,
                                                                                   const Eigen::Vector3d&                           agent_pos,
                                                                                   const double&                                    altitude)
{

  if (cloud->size() <= 0) {
    std::cout << "[RBLController]: PointCloud is empty. Can not clean ground" << std::endl;
    return nullptr;
  }

  if (params_.use_map) {
    voxelizePcl(cloud, params_.voxel_size);
  }

  // if (params_.replanner) {
  //   std::cout << "[RBLController]: Adding an artificial ground plan to the PointCloud at this height: " << agent_pos.z() - altitude << std::endl; //TODO uncoment

  //   Eigen::Vector3d patch_seed(agent_pos.x(), agent_pos.y(), agent_pos.z() - altitude);
  //   auto            ground_patch = getpointsInsideCircle(patch_seed, 3.0, params_.voxel_size);

  //   for (const auto& p : ground_patch) {
  //     cloud->emplace_back(p.x(), p.y(), p.z());
  //   }
  // }

  pcl::PointCloud<pcl::PointXYZ> temp_no_ground_cloud;
  temp_no_ground_cloud.reserve(cloud->points.size());

  for (const auto& pt : cloud->points) {
    if (pt.z > (agent_pos.z() - params_.encumbrance)) {
      temp_no_ground_cloud.push_back(pt);
    }
  }

  return std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(temp_no_ground_cloud);
}

void RBLController::voxelizePcl(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& cloud,
                                double                                           voxel_size)
{
  // std::cout << "[RBLController]: Voxelizing PointCloud to voxel size: " << voxel_size << std::endl; //TODO uncomment

  pcl::PointCloud<pcl::PointXYZ>::Ptr boost_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr boost_voxelized_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(boost_cloud);
  sor.setLeafSize(static_cast<float>(voxel_size), static_cast<float>(voxel_size), static_cast<float>(voxel_size));
  sor.filter(*boost_voxelized_cloud);

  cloud = std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(boost_voxelized_cloud.get(), [boost_voxelized_cloud](...) {});
}

std::vector<Eigen::Vector3d> RBLController::getpointsInsideCircle(const Eigen::Vector3d& center,
                                                                  const double&          radius,
                                                                  const double&          step_size)
{
  std::vector<Eigen::Vector3d> points;
  const double&                x_center = center[0];
  const double&                y_center = center[1];
  const double&                z_center = center[2];

  int x_min = static_cast<int>((x_center - radius) / step_size);
  int x_max = static_cast<int>((x_center + radius) / step_size);
  int y_min = static_cast<int>((y_center - radius) / step_size);
  int y_max = static_cast<int>((y_center + radius) / step_size);

  std::vector<double> x_coords;
  std::vector<double> y_coords;

  for (int i = x_min; i <= x_max; ++i)
    x_coords.push_back(i * step_size);

  for (int j = y_min; j <= y_max; ++j)
    y_coords.push_back(j * step_size);

  for (auto x : x_coords) {
    for (auto y : y_coords) {
      double distance = std::sqrt(std::pow((x - x_center), 2) + std::pow((y - y_center), 2));
      if (distance <= radius)
        points.push_back(Eigen::Vector3d{ x, y, z_center });
    }
  }

  return points;
}

void RBLController::pointsInsideSphere(std::vector<Eigen::Vector3d>& sphere,
                                       const Eigen::Vector3d&        center,
                                       const double&                 radius,
                                       const double&                 step_size,
                                       const double&                 altitude)
{
  double x_center = center[0];
  double y_center = center[1];
  double z_center = center[2];

  double r_low  = std::min(radius, std::max(altitude - params_.z_min, 0.));
  double r_high = std::min(radius, params_.z_max - altitude);

  int x_min = static_cast<int>((x_center - radius) / step_size);
  int x_max = static_cast<int>((x_center + radius) / step_size);
  int y_min = static_cast<int>((y_center - radius) / step_size);
  int y_max = static_cast<int>((y_center + radius) / step_size);
  int z_min = static_cast<int>((z_center - r_low) / step_size);
  int z_max = static_cast<int>((z_center + r_high) / step_size);

  std::vector<double> x_coords, y_coords, z_coords;
  for (int i = x_min; i <= x_max; ++i)
    x_coords.push_back(i * step_size);
  for (int j = y_min; j <= y_max; ++j)
    y_coords.push_back(j * step_size);
  for (int k = z_min; k <= z_max; ++k)
    z_coords.push_back(k * step_size);

  for (auto x : x_coords) {
    for (auto y : y_coords) {
      for (auto z : z_coords) {
        double distance =
            std::sqrt(std::pow((x - x_center), 2) + std::pow((y - y_center), 2) + std::pow((z - z_center), 2));
        if (distance <= radius) {
          sphere.push_back(Eigen::Vector3d(x, y, z));
        }
      }
    }
  }
}

void RBLController::partitionCellA(std::vector<Eigen::Vector3d>&                    cell_A,
                                   std::vector<Eigen::Vector3d>&                    cell_S,
                                   const Eigen::Vector3d&                           agent_pos,
                                   const std::vector<Eigen::Vector3d>&              neighbors,
                                   std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& cloud)
{

  std::vector<bool>                   remove_mask(cell_S.size(), false);
  pcl::PointCloud<pcl::PointXYZ>::Ptr boost_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud);

  std::vector<Eigen::Vector3d> plane_normals;
  std::vector<Eigen::Vector3d> plane_points;

  // check other agents
  for (const auto& neighbor : neighbors) {
    double          Delta_i_j = 2 * params_.encumbrance;
    Eigen::Vector3d tilde_p_i = Delta_i_j * (neighbor - agent_pos) / ((neighbor - agent_pos).norm()) + agent_pos;
    Eigen::Vector3d tilde_p_j = Delta_i_j * (agent_pos - neighbor) / ((agent_pos - neighbor).norm()) + neighbor;

    Eigen::Vector3d plane_norm, plane_point;
    if ((agent_pos - tilde_p_i).norm() <= (agent_pos - tilde_p_j).norm()) {
      plane_norm  = tilde_p_j - tilde_p_i;
      plane_point = tilde_p_i + params_.cwvd_rob * plane_norm;
    }
    else {
      plane_norm  = tilde_p_i - tilde_p_j;
      plane_point = tilde_p_j;
    }
    plane_normals.push_back(plane_norm);
    plane_points.push_back(plane_point);
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  bool check_cloud = false;
  if (cloud->size() > 0) {
    kdtree.setInputCloud(boost_cloud);
    check_cloud = true;
  }

  if (check_cloud) {
    std::vector<int>   k_indices(1);
    std::vector<float> k_sqr_distances(1);

    std::vector<int>   radius_indices;
    std::vector<float> radius_sqr_distances;

    pcl::PointXYZ searchPoint(agent_pos.x(), agent_pos.y(), agent_pos.z());
    if (kdtree.radiusSearch(searchPoint, radius_sensing_, radius_indices, radius_sqr_distances) > 0) {
      for (size_t i = 0; i < radius_indices.size(); ++i) {
        const auto& current_voxel = boost_cloud->points[radius_indices[i]];

        Eigen::Vector3d voxel_point(current_voxel.x, current_voxel.y, current_voxel.z);
        Eigen::Vector3d closest_p_on_vox;
        closestPointOnVoxel(closest_p_on_vox, agent_pos, voxel_point, params_.voxel_size);
        double          Delta_i_j = params_.encumbrance + sqrt(3 * pow(params_.voxel_size / 2.0, 2));
        Eigen::Vector3d tilde_p_i =
            Delta_i_j * (voxel_point - agent_pos) / (voxel_point - agent_pos).norm() + agent_pos;
        Eigen::Vector3d tilde_p_j =
            Delta_i_j * (agent_pos - voxel_point) / (agent_pos - voxel_point).norm() + voxel_point;

        Eigen::Vector3d plane_norm, plane_point;
        if ((agent_pos - tilde_p_i).norm() <= (agent_pos - tilde_p_j).norm()) {
          plane_norm  = tilde_p_j - tilde_p_i;
          plane_point = tilde_p_i + params_.cwvd_obs * plane_norm;
        }
        else {
          plane_norm  = tilde_p_i - tilde_p_j;
          plane_point = tilde_p_j;
        }
        plane_normals.push_back(plane_norm);
        plane_points.push_back(plane_point);
      }
    }
  }

  std::vector<double> plane_offsets(plane_normals.size());

  // #pragma omp parallel for
  for (int j = 0; j < static_cast<int>(plane_normals.size()); ++j) {
    plane_offsets[j] = plane_normals[j].dot(plane_points[j]);
  }


  // #pragma omp parallel for schedule(dynamic, 64)
  for (int i = 0; i < static_cast<int>(cell_S.size()); ++i) {
    if (remove_mask[i])
      continue;

    const Eigen::Vector3d& point         = cell_S[i];
    bool                   should_remove = false;

    for (size_t j = 0; j < plane_normals.size(); ++j) {
      double side = plane_normals[j].dot(point) - plane_offsets[j];
      if (side >= 0) {
        should_remove = true;
        break;
      }
    }

    if (should_remove)
      remove_mask[i] = true;
  }

  for (size_t i = 0; i < cell_S.size(); ++i) {
    if (!remove_mask[i]) {
      cell_A.push_back(cell_S[i]);
    }
  }
}

bool RBLController::partitionCellACiri(std::vector<Eigen::Vector3d>&                    cell_A,
                                       std::vector<Eigen::Vector3d>&                    cell_S,
                                       const Eigen::Vector3d&                           agent_pos,
                                       const Eigen::Vector3d&                           waypoint,
                                       const std::vector<Eigen::Vector3d>&              neighbors,
                                       std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& cloud)
{ 
  // map cloud ptr to Eigen::Matrix3Xd as input to ciri
  size_t num_points = cloud->points.size();
  if (num_points == 0) {
    std::cout << "[RBLController]: Mapping cloud ptr to Eigen::Matrix3Xd not possible, cloud is empty." << std::endl;
    return false;
  }
  Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>, 0, Eigen::Stride<4, 1>> pcl_map(reinterpret_cast<float*>(cloud->points.data()), 3, num_points);
  const Eigen::Matrix3Xf& pc = pcl_map;

  Eigen::MatrixX4f bd(6, 4); 
  // Populate the boundary matrix with plane equations
  bd <<  1,  0,  0, -(agent_pos.x() + 10),
      -1,  0,  0,  (agent_pos.x() - 10), 
       0,  1,  0, -(agent_pos.y() + 10), 
       0, -1,  0,  (agent_pos.y() - 10), 
       0,  0,  1, -(agent_pos.z() + 10), 
       0,  0, -1,  (agent_pos.z() - 10); 


  bool result = ciri_solver_->comvexDecomposition(bd, pc, agent_pos.cast<float>(), waypoint.cast<float>());

  if (result) {
    // std::cout << "[RBLController]: Convex decomposition was successful." << std::endl;
  } else {
    // std::cout << "[RBLController]: Convex decomposition failed. " << std::endl;
    return false;
  }


  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> plane_data = ciri_solver_->getPlaneData();


  std::vector<bool>                   remove_mask(cell_S.size(), false);

  std::vector<Eigen::Vector3d> plane_normals;
  std::vector<Eigen::Vector3d> plane_points;

  convertPlaneData(plane_data, plane_normals, plane_points);

  std::vector<double> plane_offsets(plane_normals.size());

  // #pragma omp parallel for
  for (int j = 0; j < static_cast<int>(plane_normals.size()); ++j) {
    plane_offsets[j] = plane_normals[j].dot(plane_points[j]);
  }

  // #pragma omp parallel for schedule(dynamic, 64)
  for (int i = 0; i < static_cast<int>(cell_S.size()); ++i) {
    if (remove_mask[i]) continue;

    const Eigen::Vector3d& point         = cell_S[i];
    bool                   should_remove = false;

    for (size_t j = 0; j < plane_normals.size(); ++j) {
      double side = plane_normals[j].dot(point) - plane_offsets[j];
      if (side >= 0) {
        should_remove = true;
        break;
      }
    }

    if (should_remove) remove_mask[i] = true;
  }

  for (size_t i = 0; i < cell_S.size(); ++i) {
    if (!remove_mask[i]) {
      cell_A.push_back(cell_S[i]);
    }
  }
  return true;
}

void RBLController::convertPlaneData(const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& plane_data, std::vector<Eigen::Vector3d>& plane_normals, std::vector<Eigen::Vector3d>& plane_points)
{
  plane_normals.clear();
  plane_points.clear();
  
  plane_normals.reserve(plane_data.size());
  plane_points.reserve(plane_data.size());

  for (const auto& plane_pair : plane_data) {
    plane_normals.push_back(plane_pair.first.cast<double>());
    plane_points.push_back(plane_pair.second.cast<double>());
  }
}


void RBLController::closestPointOnVoxel(Eigen::Vector3d&       point,
                                        const Eigen::Vector3d& agent_pos,
                                        const Eigen::Vector3d& voxel_center,
                                        const double&          voxel_size)
{
  Eigen::Vector3d half_resolution(voxel_size / 2.0, voxel_size / 2.0, voxel_size / 2.0);

  Eigen::Vector3d min_corner = voxel_center - half_resolution;
  Eigen::Vector3d max_corner = voxel_center + half_resolution;

  point.x() = std::clamp(agent_pos.x(), min_corner.x(), max_corner.x());
  point.y() = std::clamp(agent_pos.y(), min_corner.y(), max_corner.y());
  point.z() = std::clamp(agent_pos.z(), min_corner.z(), max_corner.z());
}

void RBLController::createAndPartitionCellA(std::vector<Eigen::Vector3d>&                    cell_A,
                                            std::vector<Eigen::Vector3d>&                    cell_S,
                                            const Eigen::Vector3d&                           agent_pos,
                                            const Eigen::Vector3d&                           waypoint,
                                            const std::vector<Eigen::Vector3d>&              neighbors_pos,
                                            std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& cloud,
                                            const double&                                    altitude)
{
  if (params_.only_2d) {  // 2D case
    cell_S = getpointsInsideCircle(agent_pos, params_.radius, params_.step_size);
  }
  else {  // 3D case
    pointsInsideSphere(cell_S, agent_pos, params_.radius, params_.step_size, altitude);
  }

  if (!neighbors_pos_.empty() || (cloud && cloud->size() > 0)) {
    if (params_.ciri) {
      bool success = partitionCellACiri(cell_A, cell_S, agent_pos, waypoint, neighbors_pos, cloud);
      if (!success || cell_A.size()==0) {
        // std::cout << "[RBLController]: Ciri failed. Using classic partition." << std::endl;
        partitionCellA(cell_A, cell_S, agent_pos, neighbors_pos, cloud);
      }
    } else {
      partitionCellA(cell_A, cell_S, agent_pos, neighbors_pos, cloud);
    }    
  }
  else {
    cell_A = cell_S;
  }
}

void RBLController::computeCentroid(Eigen::Vector3d&              centroid,
                                    std::vector<Eigen::Vector3d>& cell,
                                    Eigen::Vector3d&              destination,
                                    double&                       beta)
{
  std::vector<double> x_in, y_in, z_in;
  for (const auto& point : cell) {
    x_in.push_back(point[0]);
    y_in.push_back(point[1]);
    z_in.push_back(point[2]);
  }

  std::vector<double> scalar_values;
  compute_scalar_value(scalar_values, x_in, y_in, z_in, destination, beta);

  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_z = 0.0;
  double sum   = 0.0;

  for (size_t i = 0; i < x_in.size(); ++i) {
    sum_x += x_in[i] * scalar_values[i];
    sum_y += y_in[i] * scalar_values[i];
    sum_z += z_in[i] * scalar_values[i];
    sum += scalar_values[i];
  }
  centroid = Eigen::Vector3d(sum_x / sum, sum_y / sum, sum_z / sum);
}

void RBLController::compute_scalar_value(std::vector<double>&       scalar_values,
                                         const std::vector<double>& x_test,
                                         const std::vector<double>& y_test,
                                         const std::vector<double>& z_test,
                                         const Eigen::Vector3d&     destination,
                                         double                     beta)
{
  for (size_t i = 0; i < x_test.size(); ++i) {
    double distance = std::sqrt(std::pow((x_test[i] - destination[0]), 2) + std::pow((y_test[i] - destination[1]), 2) +
                                std::pow((z_test[i] - destination[2]), 2));
    double scalar_value = std::exp(-distance / beta);
    scalar_values.push_back(scalar_value);
  }
}

void RBLController::applyRules(double&                beta,
                               double&                th,
                               double&                ph,
                               Eigen::Vector3d        destination,
                               const Eigen::Vector3d  goal,
                               const Eigen::Vector3d& agent_pos,
                               const Eigen::Vector3d& c1,
                               const Eigen::Vector3d& c2,
                               const Eigen::Vector3d& c1_no_rot,
                               const double&          d1,
                               const double&          d2,
                               const double&          d3,
                               const double&          d4,
                               const double&          d5,
                               const double&          d6,
                               const double&          d7,
                               const double&          betaD,
                               const double&          beta_min,
                               const double&          dt)
{
  double current_j_x = agent_pos[0];
  double current_j_y = agent_pos[1];
  double current_j_z = agent_pos[2];

  if (!params_.only_2d) {
    // first condition
    double dist_c1_c2 = sqrt(pow((c1[0] - c2[0]), 2) + pow((c1[1] - c2[1]), 2) + pow((c1[2] - c2[2]), 2));
    double dist_current_c1 =
        sqrt(pow(current_j_x - c1[0], 2) + pow(current_j_y - c1[1], 2) + pow(current_j_z - c1[2], 2));
    if (dist_c1_c2 > d2 && dist_current_c1 < d1) {
      beta = std::max(beta - dt, beta_min);  //  std::max(beta - dt * (epsilon), beta_min);
    }
    else {
      beta = std::max(beta - dt * (beta - betaD), beta_min);
    }

    // second condition
    double dist_c1_c2_plane_xy      = sqrt(pow((c1[0] - c2[0]), 2) + pow((c1[1] - c2[1]), 2));
    double dist_current_c1_plane_xy = sqrt(pow(current_j_x - c1[0], 2) + pow(current_j_y - c1[1], 2));
    bool   dist_c1_c2_plane_xy_d4   = dist_c1_c2_plane_xy > d4;
    if (dist_c1_c2_plane_xy_d4 && dist_current_c1_plane_xy < d3) {
      th = std::min(th + dt, M_PI / 2);
    }
    else {
      th = std::max(0.0, th - 2 * dt);
    }


    // third condition
    if (th == M_PI / 2 &&
        sqrt(pow((current_j_x - c1_no_rot[0]), 2) + pow((current_j_y - c1_no_rot[1]), 2)) > dist_current_c1_plane_xy) {
      th = 0;
    }

    double dist_c1_c2_z      = fabs(c1[2] - c2[2]);
    double dist_current_c1_z = fabs(current_j_z - c1[2]);
    double to_c2_z           = c2[2] - current_j_z;

    double dist_current_c2_plane_xy = sqrt(pow(current_j_x - c2[0], 2) + pow(current_j_y - c2[1], 2));

    if (params_.use_z_rule) {
      if (dist_c1_c2_z > d5 && dist_current_c1_z < d6 ||
          (dist_current_c2_plane_xy - dist_current_c1_plane_xy) > d7) {  // modify phi up down based on some if
        double direction_to_goal = std::atan2(goal[0] - current_j_x, goal[1] - current_j_y);  // [0, 2pi)
        if (direction_to_goal < 0) {
          direction_to_goal += 2 * M_PI;
        }
        // linear mapping
        double direction_influence = (direction_to_goal / M_PI) - 1.0;

        double w1 = 0.7;  // Weight for to_c2_z TODO LOAD but this ok
        double w2 = 0.3;  // Weight for delta_c1_z

        // Calculate weighted average
        double combined_influence = (w1 * to_c2_z + w2 * direction_influence) / (w1 + w2);

        if (combined_influence > 0) {
          ph = std::min(M_PI_4, ph + dt);
        }
        else {
          ph = std::max(-M_PI_4, ph - dt);
        }
      }
      else {  // converge back to ph = 0
        if (ph > 0.0) {
          ph = std::max(0.0, ph - dt);
        }
        else if (ph < 0.0) {
          ph = std::min(0.0, ph + dt);
        }
        else {
          ph = 0.0;
        }
      }

      // check if any vertical avoidance && if if vertical adjust makes drone further from uav/obstacle
      double dist_current_c2_z = fabs(current_j_z - c1[2]);
      if (fabs(ph) == M_PI_4 && dist_current_c2_z > fabs(current_j_z - c1_no_rot[2])) {
        ph = 0;
      }
    }


    double dx   = goal[0] - current_j_x;
    double dy   = goal[1] - current_j_y;
    double dz   = goal[2] - current_j_z;
    double dist = sqrt(dx * dx + dy * dy + dz * dz);

    double theta = atan2(dy, dx);    //!! azimuthal angle in xy plane
    double phi   = acos(dz / dist);  // polar angel from the z axis

    double new_theta = theta - th;
    double new_phi   = phi + ph;

    destination[0] = current_j_x + dist * sin(new_phi) * cos(new_theta);
    destination[1] = current_j_y + dist * sin(new_phi) * sin(new_theta);
    destination[2] = current_j_z + dist * cos(new_phi);
  }
  else {
    // first condition
    double dist_c1_c2 = sqrt(pow((c1[0] - c2[0]), 2) + pow((c1[1] - c2[1]), 2));
    if (dist_c1_c2 > d2 && sqrt(pow((current_j_x - c1[0]), 2) + pow((current_j_y - c1[1]), 2)) < d1) {
      beta = std::max(beta - dt, beta_min);
    }
    else {
      beta = beta - dt * (beta - betaD);
    }

    // second condition
    bool dist_c1_c2_d4 = dist_c1_c2 > d4;
    if (dist_c1_c2_d4 && sqrt(pow((current_j_x - c1[0]), 2) + pow((current_j_y - c1[1]), 2)) < d3) {
      th = std::min(th + dt, M_PI / 2);
    }
    else {
      th = std::max(0.0, th - dt);
    }

    // third condition
    if (th == M_PI / 2 && sqrt(pow((current_j_x - c1_no_rot[0]), 2) + pow((current_j_y - c1_no_rot[1]), 2)) >
                              sqrt(pow((current_j_x - c1[0]), 2) + pow((current_j_y - c1[1]), 2))) {
      th = 0;
    }

    double angle     = atan2(goal[1] - current_j_y, goal[0] - current_j_x);
    double new_angle = angle - th;
    double distance  = sqrt(pow((goal[0] - current_j_x), 2) + pow((goal[1] - current_j_y), 2));
    destination[0]   = current_j_x + distance * cos(new_angle);
    destination[1]   = current_j_y + distance * sin(new_angle);
  }
}

Eigen::Vector3d RBLController::determineWaypoint(const std::vector<Eigen::Vector3d>& path, 
                                                 const Eigen::Vector3d& agent_pos,
                                                 const Eigen::Vector3d& goal)
{
  double min_dist_sq = std::numeric_limits<double>::max();
  int closest_point_index = -1;

  for (size_t i = 0; i < path.size(); ++i) {
    double dist_sq = (path[i] - agent_pos).squaredNorm();
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_point_index = i;
    }
  }

  if (closest_point_index == -1 || closest_point_index >= path.size() - 1) {
    return path.back(); 
  }

  double dist_agent_goal = (goal - agent_pos).norm();
  Eigen::Vector3d closest_point = path[closest_point_index];
  Eigen::Vector3d next_point = path[closest_point_index + 1];
  Eigen::Vector3d direction_vector = (next_point - closest_point) / (next_point - closest_point).norm();

  Eigen::Vector3d waypoint;
  // if (params_.ciri) {
  //   waypoint = next_point;
  // } else {
  //   waypoint = closest_point + direction_vector * std::min(params_.radius, dist_agent_goal);
  // }
  waypoint = closest_point + direction_vector * std::min(params_.radius, dist_agent_goal);
  
  return waypoint;
}

void RBLController::determineNextRef(mrs_msgs::Reference&           p_ref,
                                     const Eigen::Vector3d&         agent_pos,
                                     const Eigen::Vector3d&         goal,
                                     const Eigen::Vector3d&         c1,
                                     const Eigen::Vector3d&         rpy,
                                     const std::vector<Eigen::Vector3d>& path)
{
  if (params_.limited_fov) {
    double desired_heading;
    if (params_.replanner) {
      desired_heading = determineYaw(agent_pos, path, rpy); //using replanner determin heading based on path
    } else {
      desired_heading = std::atan2(c1[1] - agent_pos[1], c1[0] - agent_pos[0]); //rotate towards the calculated centroid
    }
    
    double diff       = std::fmod(desired_heading - rpy[2] + M_PI, 2 * M_PI) - M_PI;
    double difference = (diff < -M_PI) ? diff + 2 * M_PI : diff;
    if (std::abs(difference) < M_PI / 2) { //+-90 deg
      p_ref.position.x = c1[0];
      p_ref.position.y = c1[1];
      p_ref.position.z = c1[2];
    } else {
      p_ref.position.x = agent_pos[0];
      p_ref.position.y = agent_pos[1];
      p_ref.position.z = agent_pos[2];
    }
    p_ref.heading = desired_heading;

    if ((agent_pos - goal).norm() <= 0.2) { //Arived at goal pos
      p_ref.heading = rpy[2]; //keep the same heading
    } else {
      p_ref.heading = desired_heading;
    }
  }
  else {
    p_ref.position.x = c1[0];
    p_ref.position.y = c1[1];
    p_ref.position.z = c1[2];
  }
}

double RBLController::determineYaw(const Eigen::Vector3d& agent_pos, const std::vector<Eigen::Vector3d>& path, const Eigen::Vector3d& rpy)
{
  const double max_yaw_change = M_PI / 6.0; 
  double current_yaw = rpy[2];
  double min_dist_sq = std::numeric_limits<double>::max();
  int closest_point_index = -1;

  for (size_t i = 0; i < path.size(); ++i) {
    double dist_sq = (path[i] - agent_pos).squaredNorm();
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_point_index = i;
    }
  }

  Eigen::Vector3d closest_point = path[closest_point_index];

  if (closest_point_index == -1 || closest_point_index >= path.size() - 1) {
    return current_yaw; 
  }

  const double look_ahead_dist_sq = 5.0 * 5.0; 

  Eigen::Vector3d target_point = path[closest_point_index];
  double accumulated_dist_sq = 0.0;
  double yaw_diff_max = 0.0;
  double yaw_diff_min = 0.0;

  bool yaw_diff_max_first = false;
  bool yaw_diff_min_first = false;

  for (size_t i = closest_point_index + 1; i < path.size(); ++i) {
    accumulated_dist_sq += (path[i] - path[i-1]).squaredNorm();
    Eigen::Vector3d direction_vector = path[i] - agent_pos;
    double yaw_to_point = std::atan2(direction_vector.y(), direction_vector.x());
    double yaw_diff = yaw_to_point - current_yaw;
    yaw_diff_max = std::max(yaw_diff_max, yaw_diff);
    yaw_diff_min = std::min(yaw_diff_min, yaw_diff);

    if (yaw_diff_max>max_yaw_change && !yaw_diff_min_first) {
      yaw_diff_max_first = true;
    }

    if (-yaw_diff_min<-max_yaw_change && !yaw_diff_max_first) {
      yaw_diff_min_first = true;
    }

    if (accumulated_dist_sq > look_ahead_dist_sq || (i == path.size()-1)) {
      break;
    }
  }

  double final_yaw;
  if (yaw_diff_max_first) {
    final_yaw = current_yaw + yaw_diff_max;
  } else if (yaw_diff_min_first) {
    final_yaw = current_yaw + yaw_diff_min;
  } else { //final goal
    Eigen::Vector3d direction_vector = path.back() - agent_pos;
    final_yaw = std::atan2(direction_vector.y(), direction_vector.x());
  }

  final_yaw = normalizeAngle(final_yaw);

  return final_yaw;
}

double RBLController::normalizeAngle(double angle)
{
  if (angle > M_PI) {
    angle -= 2 * M_PI;
  } else if (angle < -M_PI) {
    angle += 2 * M_PI;
  }
  return angle;
}