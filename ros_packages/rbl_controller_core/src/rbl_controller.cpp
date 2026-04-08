#include "rbl_controller_core/rbl_controller.h"

RBLController::RBLController(const RBLParams& params) : params_(params)  // //{
{
  radius_sensing_ = params_.radius / params_.cwvd_obs + sqrt((params_.voxel_size / 2) * (params_.voxel_size / 2));
  beta_           = params_.beta_min;
  if (params.replanner) {
    ReplannerParams replanner_params;
    replanner_params.encumbrance        = params.encumbrance;
    replanner_params.voxel_size         = params.voxel_size;
    replanner_params.map_width          = 30.0;
    replanner_params.map_height         = 10.0;
    replanner_params.weight_safety      = 1.0;
    replanner_params.weight_deviation   = 100.0;
    replanner_params.inflation_bonus    = params.inflation_bonus;
    replanner_params.replanner_vox_size = 0.4;
    replanner_params.replanner_freq     = 0.5;  //[Hz]

    rbl_replanner_ = std::make_shared<RBLReplanner>(replanner_params);
  }

  if (params.ciri) {
    ciriParams ciri_params;
    ciri_params.epsilon   = 1e-6;
    ciri_params.inflation = params.encumbrance + params.voxel_size;
    ciri_solver_          = std::make_shared<CIRI>(ciri_params);
  }
}  // //}

void RBLController::setCurrentPosition(const Eigen::Vector3d& point)  // //{
{
  agent_pos_ = point;
  if (!params_.use_garmin_alt) {
    altitude_ = agent_pos_.z();
  }
}  // //}

void RBLController::setCurrentVelocity(const Eigen::Vector3d& point)  // //{
{
  agent_vel_ = point;
}  // //}

void RBLController::setGroupStates(const std::vector<State>& states)  // //{
{
  group_states_ = states;
}  // //}

void RBLController::setPCL(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud)  // //{
{
  cloud_ = cloud;
}  // //}
   //

void RBLController::setPCL1(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud)  // //{
{
  cloud_obs_ = cloud;
}  // //}
   
void RBLController::setGoal(const Eigen::Vector3d& point)  // //{
{
  goal_        = point;
  destination_ = point;
  ph_          = 0.0;
  th_          = 0.0;
}  // //}

void RBLController::setBetaD(double beta)
{
  params_.betaD = beta;
}

void RBLController::setAltitude(const double& alt)  // //{
{
  altitude_ = alt;
}  // //}

void RBLController::setRollPitchYaw(const Eigen::Vector3d& rpy)  // //{
{
  rpy_ = rpy;
}  // //}

bool RBLController::inputsHealthy(const Eigen::Vector3d&                           agent_pos,
                                  const Eigen::Vector3d&                           agent_vel,
                                  const std::vector<State>&                        group_states,
                                  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud,
                                  Eigen::Vector3d&                                 goal,
                                  double&                                          altitude,
                                  Eigen::Vector3d&                                 rpy)
{
  bool healthy = true;

  auto validVector = [](const Eigen::Vector3d& v) -> bool { return v.allFinite(); };

  // Helper to rate-limit messages
  static std::map<std::string, std::chrono::steady_clock::time_point> last_print;
  auto                                                                now = std::chrono::steady_clock::now();
  auto printLimited = [&](const std::string& key, const std::string& msg) {
    constexpr double interval = 1.0;  // seconds
    auto             it       = last_print.find(key);
    if (it == last_print.end() || std::chrono::duration<double>(now - it->second).count() > interval) {
      std::cerr << msg << std::endl;
      last_print[key] = now;
    }
  };

  // Checks
  if (!validVector(agent_pos)) {
    printLimited("agent_pos",
                 "[RBLController]: Invalid agent_pos (NaN/Inf): " + std::to_string(agent_pos.x()) + ", " +
                     std::to_string(agent_pos.y()) + ", " + std::to_string(agent_pos.z()));
    healthy = false;
  }

  if (!validVector(agent_vel)) {
    printLimited("agent_vel", "[RBLController]: Invalid agent_vel (NaN/Inf)");
    healthy = false;
  }

  if (!validVector(goal)) {
    printLimited("goal", "[RBLController]: Invalid goal (NaN/Inf)");
    healthy = false;
  }

  if (!validVector(rpy)) {
    printLimited("rpy", "[RBLController]: Invalid rpy (NaN/Inf)");
    healthy = false;
  }

  if (!std::isfinite(altitude) || altitude < 0.0) {
    printLimited("altitude", "[RBLController]: Invalid altitude (non-finite or negative)");
    healthy = false;
  }

  for (const auto& state : group_states) {
    const auto& pos = state.position;
    const auto& vel = state.velocity;
    if (!validVector(pos)) {
      printLimited("neighbor_pos", "[RBLController]: One or more neighbors have invalid positions");
      healthy = false;
      break;
    }
    if (!validVector(vel)) {
      printLimited("neighbor_vel", "[RBLController]: One or more neighbors have invalid velocities");
      healthy = false;
      break;
    }
  }

  if (!cloud) {
    printLimited("cloud_null", "[RBLController]: Point cloud pointer is null");
    healthy = false;
  }
  else if (cloud->empty()) {
    printLimited("cloud_empty", "[RBLController]: Point cloud is empty");
    healthy = false;
  }

  if (!healthy) {
    printLimited("summary", "[RBLController]: One or more input checks failed!");
  }

  return healthy;
}

std::optional<mrs_msgs::msg::Reference> RBLController::getNextRef()  // //{
{
  mrs_msgs::msg::Reference p_ref;

  /* if (!inputsHealthy(agent_pos_, agent_vel_, group_states_, cloud_, goal_, altitude_, rpy_)) { */
  /*   std::cout << "[RBLController]: Inputs are not ok. Cannot return next reference" << std::endl; */
  /*   return std::nullopt; */
  /* } */

  // cloud_ = getGroundCleanCloud(cloud_, agent_pos_, altitude_);
  // cloud_obs_ = getGroundCleanCloud(cloud_obs_, agent_pos_, altitude_);
  if (!cloud_) {
    return std::nullopt;
  }

  if (params_.replanner) {

    // Launch replanner only if not already running
    if (rbl_replanner_->replanTimer() &&
        (!replanner_future_.valid() ||
         replanner_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)) {

      replanner_future_ = std::async(std::launch::async, [this]() {
        rbl_replanner_->setAltitude(altitude_);
        rbl_replanner_->setCurrentPosition(agent_pos_);
        rbl_replanner_->setGoal(goal_);
        rbl_replanner_->setPCL(cloud_obs_);

        auto new_path = rbl_replanner_->plan();

        std::lock_guard<std::mutex> lock(replanner_mutex_);
        inflated_map_ = rbl_replanner_->getInflatedCloud();
        return new_path;
      });
    }

    // Consume planner result if ready (non-blocking)
    if (replanner_future_.valid() &&
        replanner_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {

      std::lock_guard<std::mutex> lock(replanner_mutex_);
      path_ = replanner_future_.get();   // latch new path
    }

    // Use path if available, otherwise keep moving
    if (!path_.empty()) {
      waypoint_fixed_distance_ = determineWaypointFixedDistance(path_, agent_pos_, goal_);
      waypoint_                = determineWaypoint(path_, agent_pos_, goal_, waypoint_);
      destination_             = waypoint_;
    } else {
      // Fallback behavior — DO NOT return
      destination_ = goal_;   // or keep previous destination_
    }
  }
  // if (params_.replanner) {
  //   // Trigger replanner asynchronously
  //   if (rbl_replanner_->replanTimer()) {
  //     replanner_future_ = std::async(std::launch::async, [this]() {
  //       rbl_replanner_->setAltitude(altitude_);
  //       rbl_replanner_->setCurrentPosition(agent_pos_);
  //       rbl_replanner_->setGoal(goal_);
  //       rbl_replanner_->setPCL(cloud_obs_);
  //       auto                        new_path = rbl_replanner_->plan();
  //       std::lock_guard<std::mutex> lock(replanner_mutex_);
  //       inflated_map_ = rbl_replanner_->getInflatedCloud();
  //       return new_path;
  //     });
  //   }

  //   // Check if replanner finished and update path_
  //   if (replanner_future_.valid() &&
  //       replanner_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
  //     std::lock_guard<std::mutex> lock(replanner_mutex_);
  //     path_ = replanner_future_.get();
  //   }

  //   if (path_.empty()) {
  //     p_ref = pRefAgent(agent_pos_, rpy_[2]);
  //     return p_ref;
  //   }

  //   waypoint_fixed_distance_ = determineWaypointFixedDistance(path_, agent_pos_, goal_);
  //   waypoint_                = determineWaypoint(path_, agent_pos_, goal_, waypoint_);
  //   destination_             = waypoint_;
  // }

  std::vector<Eigen::Vector3d> group_positions;
  for (const auto& state : group_states_) {
    group_positions.push_back(state.position);
  }

  // Partitioning logic
  cell_A_.clear();
  sensed_cell_A_.clear();
  cell_S_.clear();
  createAndPartitionCellA(cell_A_,
                          sensed_cell_A_,
                          cell_S_,
                          plane_normals_,
                          plane_points_,
                          agent_pos_,
                          rpy_,
                          waypoint_,
                          group_positions,
                          cloud_,
                          altitude_,
                          c1_,
                          seed_b_,
                          threshold_active_);


  std::vector<Eigen::Vector3d> emptyVec;
  if (params_.replanner) {
    computeCentroid(c1_full_,
                    agent_pos_,
                    agent_vel_,
                    cell_A_,
                    plane_normals_,
                    plane_points_,
                    destination_,
                    waypoint_fixed_distance_,
                    beta_,
                    flag_threshold,
                    threshold_active_);
    computeCentroid(c2_,
                    agent_pos_,
                    agent_vel_,
                    cell_S_,
                    emptyVec,
                    emptyVec,
                    destination_,
                    waypoint_fixed_distance_,
                    beta_,
                    flag_threshold,
                    threshold_active_);
    computeCentroid(c1_no_rot_,
                    agent_pos_,
                    agent_vel_,
                    cell_A_,
                    plane_normals_,
                    plane_points_,
                    waypoint_,
                    waypoint_fixed_distance_,
                    beta_,
                    flag_threshold,
                    threshold_active_);
    applyRules(beta_,
               th_,
               ph_,
               destination_,
               seed_b_,
               waypoint_,
               agent_pos_,
               c1_,
               c2_,
               c1_no_rot_,
               params_.d1,
               params_.d2,
               params_.d3,
               params_.d4,
               params_.d5,
               params_.d6,
               params_.d7,
               params_.betaD,
               params_.beta_min,
               params_.dt);
  }
  else {
    computeCentroid(c1_full_,
                    agent_pos_,
                    agent_vel_,
                    cell_A_,
                    plane_normals_,
                    plane_points_,
                    destination_,
                    waypoint_fixed_distance_,
                    beta_,
                    flag_threshold,
                    threshold_active_);
    computeCentroid(c2_,
                    agent_pos_,
                    agent_vel_,
                    cell_S_,
                    emptyVec,
                    emptyVec,
                    destination_,
                    waypoint_fixed_distance_,
                    beta_,
                    flag_threshold,
                    threshold_active_);
    computeCentroid(c1_no_rot_,
                    agent_pos_,
                    agent_vel_,
                    cell_A_,
                    plane_normals_,
                    plane_points_,
                    goal_,
                    waypoint_fixed_distance_,
                    beta_,
                    flag_threshold,
                    threshold_active_);
    applyRules(beta_,
               th_,
               ph_,
               destination_,
               seed_b_,
               goal_,
               agent_pos_,
               c1_,
               c2_,
               c1_no_rot_,
               params_.d1,
               params_.d2,
               params_.d3,
               params_.d4,
               params_.d5,
               params_.d6,
               params_.d7,
               params_.betaD,
               params_.beta_min,
               params_.dt);
  }
  // Eigen::Vector3d c1_full_cell = c1_;
  // if (params_.move_centroid_to_sensed_cell) {
    c1_ = movePointToCell(c1_full_, sensed_cell_A_);
  // }
  // if c1_ is very close to uav.
  determineNextRef(p_ref, agent_pos_, waypoint_, goal_, c1_, c1_full_, rpy_, path_);

  return p_ref;
}  // //}

Eigen::Vector3d RBLController::getGoal()  // //{
{
  return goal_;
}  // //}

Eigen::Vector3d RBLController::getWaypoint()  // //{
{
  return waypoint_;
}  // //}

Eigen::Vector3d RBLController::getCurrentPosition()  // //{
{
  return agent_pos_;
}  // //}

Eigen::Vector3d RBLController::getCurrentVelocity()  // //{
{
  return agent_vel_;
}  // //}

Eigen::Vector3d RBLController::getCentroid()  // //{
{
  return c1_;
}  // //}

Eigen::Vector3d RBLController::getSeedB()  // //{
{
  return seed_b_;
}  // //}

std::vector<Eigen::Vector3d> RBLController::getCellA()  // //{
{
  return cell_A_;
}  // //}

std::vector<Eigen::Vector3d> RBLController::getSensedCellA()
{
  return sensed_cell_A_;
}

std::vector<Eigen::Vector3d> RBLController::getInflatedMap()  // //{
{
  return inflated_map_;
}  // //}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> RBLController::getPCL()
{
  return cloud_;
}

std::vector<Eigen::Vector3d> RBLController::getPath()  // //{
{
  if (params_.replanner) {
    return path_;
  }
  return {};
}  // //}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>
RBLController::getGroundCleanCloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud,  // //{
                                   const Eigen::Vector3d&                           agent_pos,
                                   const double&                                    altitude)
{
  (void)altitude;     // TODO - currently unused

  if (cloud->size() <= 0) {
    std::cout << "[RBLController]: PointCloud is empty. Can not clean ground" << std::endl;
    return nullptr;
  }

  if (params_.downsample_pcl) {
    cloud = downSamplePcl(cloud, params_.voxel_size);
  }

  // if (params_.replanner) {
  //   std::cout << "[RBLController]: Adding an artificial ground plan to the PointCloud at this height: " <<
  //   agent_pos.z() - altitude << std::endl; //TODO uncoment

  //   Eigen::Vector3d patch_seed(agent_pos.x(), agent_pos.y(), agent_pos.z() - altitude);
  //   auto            ground_patch = getpointsInsideCircle(patch_seed, 3.0, params_.voxel_size);

  //   for (const auto& p : ground_patch) {
  //     cloud->emplace_back(p.x(), p.y(), p.z());
  //   }
  // }

  pcl::PointCloud<pcl::PointXYZI> temp_no_ground_cloud;
  temp_no_ground_cloud.reserve(cloud->points.size());

  for (const auto& pt : cloud->points) {
    if (pt.z > (agent_pos.z() - params_.encumbrance)) {
      temp_no_ground_cloud.push_back(pt);
    }
  }

  return std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(temp_no_ground_cloud);
}  // //}

std::vector<Eigen::Vector3d> RBLController::getpointsInsideCircle(const Eigen::Vector3d& center,  // //{
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
}  // //}

void RBLController::pointsInsideSphere(std::vector<Eigen::Vector3d>& sphere,  // //{
                                       const Eigen::Vector3d&        center,
                                       const double&                 radius,
                                       const double&                 step_size,
                                       const double&                 altitude)
{
  double x_center = center[0];
  double y_center = center[1];
  double z_center = center[2];
  double r_low, r_high;
  if (params_.use_garmin_alt) {
    r_low  = std::min(radius, std::max(altitude - params_.z_min, 0.));
    r_high = std::min(radius, params_.z_max - altitude);
  }
  else {
    r_low  = std::min(radius, std::max(z_center - params_.z_min, 0.));
    r_high = std::min(radius, params_.z_max - z_center);
  }

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
}  // //}

void RBLController::partitionCellA(std::vector<Eigen::Vector3d>&                    cell_A,  // //{
                                   std::vector<Eigen::Vector3d>&                    cell_S,
                                   std::vector<Eigen::Vector3d>&                    plane_normals,
                                   std::vector<Eigen::Vector3d>&                    plane_points,
                                   const Eigen::Vector3d&                           agent_pos,
                                   const std::vector<Eigen::Vector3d>&              neighbors,
                                   std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud)
{
  // (void)neighbors;     // TODO - currently unused

  std::vector<bool>                   remove_mask(cell_S.size(), false);
  pcl::PointCloud<pcl::PointXYZI>::Ptr boost_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*cloud);

  plane_normals.clear();
  plane_points.clear();

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

  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;

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

    pcl::PointXYZI searchPoint;
    searchPoint.x = agent_pos.x();
    searchPoint.y = agent_pos.y();
    searchPoint.z = agent_pos.z();
    searchPoint.intensity = 1.0f;
    // pcl::PointXYZI searchPoint(agent_pos.x(), agent_pos.y(), agent_pos.z(), 1.0f);
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
}  // //}


bool RBLController::partitionCellACiri(std::vector<Eigen::Vector3d>&                    cell_A,  // //{
                                       std::vector<Eigen::Vector3d>&                    cell_S,
                                       std::vector<Eigen::Vector3d>&                    plane_normals,
                                       std::vector<Eigen::Vector3d>&                    plane_points,
                                       const Eigen::Vector3d&                           agent_pos,
                                       const Eigen::Vector3d&                           waypoint,
                                       const std::vector<Eigen::Vector3d>&              neighbors,
                                       std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud,
                                       Eigen::Vector3d&                                 c1,
                                       Eigen::Vector3d&                                 seed_b,
                                       bool&                                            threshold_active)
{
  (void)waypoint;     // TODO - currently unused
  (void)neighbors;     // TODO - currently unused
  (void)threshold_active;     // TODO - currently unused

  if (!ciri_solver_) {
    std::cout << "[RBLController]: Ciri solver is not initialized." << std::endl;
    return false;
  }

  // map cloud ptr to Eigen::Matrix3Xd as input to ciri
  size_t num_points = cloud->points.size();
  if (num_points == 0) {
    std::cout << "[RBLController]: Mapping cloud ptr to Eigen::Matrix3Xd not possible, cloud is empty." << std::endl;
    return false;
  }
  // Safe copy of XYZ from PointXYZI to Eigen::Matrix3Xf
  Eigen::Matrix3Xf pc(3, num_points);
  for (size_t i = 0; i < num_points; ++i) {
    pc(0, i) = cloud->points[i].x;
    pc(1, i) = cloud->points[i].y;
    pc(2, i) = cloud->points[i].z;
  }
  // Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>, 0, Eigen::Stride<4, 1>> pcl_map(
  //     reinterpret_cast<float*>(cloud->points.data()),
  //     3,
  //     num_points);
  // const Eigen::Matrix3Xf& pc = pcl_map;

  Eigen::MatrixX4f bd(6, 4);
  // Populate the boundary matrix with plane equations
  // bd <<  1,  0,  0, -(agent_pos.x() + 10),
  //     -1,  0,  0,  (agent_pos.x() - 10),
  //      0,  1,  0, -(agent_pos.y() + 10),
  //      0, -1,  0,  (agent_pos.y() - 10),
  //      0,  0,  1, -(agent_pos.z() + 10),
  //      0,  0, -1,  (agent_pos.z() - 10);
  bd << 1, 0, 0, -(agent_pos.x() + params_.radius + 1.0), -1, 0, 0, (agent_pos.x() - params_.radius - 1.0), 0, 1, 0,
      -(agent_pos.y() + params_.radius + 1.0), 0, -1, 0, (agent_pos.y() - params_.radius - 1.0), 0, 0, 1,
      -(agent_pos.z() + params_.radius + 1.0), 0, 0, -1, (agent_pos.z() - params_.radius - 1.0);

  // double dist_agent_c1 = (c1 - agent_pos).norm();
  // Eigen::Vector3d direction_vec = (agent_pos - c1)/dist_agent_c1;
  // Eigen::Vector3d direction_vec;
  // direction_vec[0] = cos(p_ref.heading);
  // direction_vec[1] = sin(p_ref.heading);
  // direction_vec[2] = 0;
  int                                                      segments = 5;
  bool                                                     result   = false;
  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> plane_data;

  // std::cout << "[RBLController]: seed_b: " << (seed_b - agent_pos).norm() << std::endl;
  // seed_b[1]<<" , "<< seed_b[2] << std::endl; std::cout << "[RBLController]: c1: " << c1[0] <<" , "<< c1[1]<<" , "<<
  // c1[2] << std::endl; Eigen::Vector3d seed_b;
  //

  if (!init_) {
    c1 = agent_pos; 
    seed_b = agent_pos; 
    init_= true;
    waypoint_ = agent_pos;
  }
  // std::cout << "[RBLController]: threshold_active " << threshold_active << std::endl;
  // std::cout << "[RBLController]: seed_b: " << (seed_b - agent_pos).norm() <<  std::endl;
  // std::cout << "[RBLController]: c1: " << c1[0] <<" , "<< c1[1]<<" , "<< c1[2] << std::endl;
  // Eigen::Vector3d seed_b;
    // Eigen::Vector3d v = c1 - seed_b;
    // double n = v.norm();
    double eps = 1e-8;
    
    // seed_b = seed_b + 2 * params_.dt * v / (n + eps); 
    seed_b = c1;


  double dist_agent_seed = (seed_b - agent_pos).norm();
  Eigen::Vector3d direction_vec = (agent_pos - seed_b)/(dist_agent_seed+eps);
  const Eigen::Vector3d base_seed = seed_b;
  const double          step_dist = dist_agent_seed / static_cast<double>(segments);

  //   for (int i = 1; i < segments; ++i) {
  for (int i = 0; i < segments; ++i) {
    const Eigen::Vector3d candidate_seed = base_seed + static_cast<double>(i) * direction_vec * step_dist;
    // Eigen::Vector3d seed_b = agent_pos +  direction_vec * (params_.radius/i);
    result     = ciri_solver_->comvexDecomposition(bd, pc, agent_pos.cast<float>(), candidate_seed.cast<float>());
    plane_data = ciri_solver_->getPlaneData();
    
    if (result) {
      seed_b = candidate_seed;
      break;
    }
    else {
      // std::cout << "[RBLController]: Moving seed closer to the uav for ciri." << std::endl;
    }
   }

  if (result) {
    // std::cout << "[RBLController]: Convex decomposition was successful." << std::endl;
  }
  else {
    // std::cout << "[RBLController]: Convex decomposition failed. " << std::endl;
    return false;
  }


  // plane_data = ciri_solver_->getPlaneData();

  // if (plane_data.size() > 50) {
  //   std::cout << "[RBLController]: Recieved planes from ciri solver: " << plane_data.size() << std::endl;
  // }

  std::vector<bool> remove_mask(cell_S.size(), false);

  plane_normals.clear();
  plane_points.clear();

  convertPlaneData(plane_data, plane_normals, plane_points, agent_pos);

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
  return true;
}  // //}

void RBLController::convertPlaneData(const std::vector<std::pair<Eigen::Vector3f,
                                                                 Eigen::Vector3f>>& plane_data,
                                     std::vector<Eigen::Vector3d>&                  plane_normals,
                                     std::vector<Eigen::Vector3d>&                  plane_points,
                                     const Eigen::Vector3d&                         agent_pos)  // //{
{
  plane_normals.clear();
  plane_points.clear();

  plane_normals.reserve(plane_data.size());
  plane_points.reserve(plane_data.size());

  for (const auto& plane_pair : plane_data) {
    // plane_normals.push_back(plane_pair.first.cast<double>());
    // plane_points.push_back(plane_pair.second.cast<double>());
    Eigen::Vector3d normal_d    = plane_pair.first.cast<double>();
    Eigen::Vector3d point_d     = plane_pair.second.cast<double>();
    Eigen::Vector3d agent_vec   = agent_pos - point_d;
    double          dot_product = agent_vec.dot(normal_d);

    if (dot_product > 0) {
      normal_d = -normal_d;
    }

    plane_normals.push_back(normal_d);
    plane_points.push_back(point_d);
  }
}  // //}

void RBLController::closestPointOnVoxel(Eigen::Vector3d&       point,  // //{
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
}  // //}

void RBLController::createAndPartitionCellA(std::vector<Eigen::Vector3d>&                    cell_A,  // //{
                                            std::vector<Eigen::Vector3d>&                    sensed_cell_A,
                                            std::vector<Eigen::Vector3d>&                    cell_S,
                                            std::vector<Eigen::Vector3d>&                    plane_normals,
                                            std::vector<Eigen::Vector3d>&                    plane_points,
                                            const Eigen::Vector3d&                           agent_pos,
                                            const Eigen::Vector3d&                           rpy,
                                            const Eigen::Vector3d&                           waypoint,
                                            const std::vector<Eigen::Vector3d>&              neighbors_pos,
                                            std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud,
                                            const double&                                    altitude,
                                            Eigen::Vector3d&                                 c1,
                                            Eigen::Vector3d&                                 seed_b,
                                            bool&                                            threshold_active)
{
std::vector<Eigen::Vector3d>                     cell_B;
std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud_high_intensity(new pcl::PointCloud<pcl::PointXYZI>());
std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud_low_intensity(new pcl::PointCloud<pcl::PointXYZI>());

constexpr float INTENSITY_THRESH = 0.99f;

for (const auto& p : cloud->points) {
  if (p.intensity >= INTENSITY_THRESH) {
    cloud_high_intensity->points.push_back(p);
  } else {
    cloud_low_intensity->points.push_back(p);
  }
}

cloud_high_intensity->width = static_cast<std::uint32_t>(cloud_high_intensity->points.size());
cloud_high_intensity->height = 1;
cloud_high_intensity->is_dense = true;

cloud_low_intensity->width = static_cast<std::uint32_t>(cloud_low_intensity->points.size());
cloud_low_intensity->height = 1;
cloud_low_intensity->is_dense = true;
// std::cout << "cloud_high_intensity: " << cloud_high_intensity->points.size() << std::endl;
// std::cout << "cloud_low_intensity: " << cloud_low_intensity->points.size() << std::endl;

// cloud_high_intensity->width  = cloud_high_intensity->points.size();
// cloud_high_intensity->height = 1;

// cloud_low_intensity->width  = cloud_low_intensity->points.size();
// cloud_low_intensity->height = 1;


  // if (low_intensity_close) {
  //   // std::cout << "1" << std::endl;
  //   seed_b = c1;   
  //   params_.ciri = false;
  //   params_.boundary_threshold = 1.0;
  // }
  // else {
  //   // std::cout << "2" << std::endl;
  //   seed_b = c1;
  //   params_.ciri = true;
  //   params_.boundary_threshold = 0.1;
  // }
  // if (agent_vel_.norm() < 0.5){
  //   seed_b = c1;
  //   params_.ciri = true;
  //   params_.boundary_threshold = 0.1;
  //   std::cout << "slow: "<< agent_vel_.norm() << std::endl;
  // }
  if (params_.only_2d) {  // 2D case
    cell_S = getpointsInsideCircle(agent_pos, params_.radius, params_.step_size);
  }
  else {  // 3D case
    pointsInsideSphere(cell_S, agent_pos, params_.radius, params_.step_size, altitude);
  }
  if (group_states_.empty()) {
      std::cout << "[RBLController]: group states empty." << std::endl;
  }
  // if (!group_states_.empty() || (cloud && cloud->size() > 0)) {
    if (params_.ciri) {
      // std::cout << "[RBLController]: cell_b "<< cell_B.size() << std::endl;
      partitionCellA(cell_B, cell_S, plane_normals, plane_points, agent_pos, neighbors_pos, cloud_high_intensity);
      /* partitionCellA(cell_B, cell_S, plane_normals, plane_points, agent_pos, group_positions, cloud_high_intensity); */
      // std::cout << "[RBLController]: cell_b1 "<< cell_B.size() << std::endl;
      bool success = false;
      if (!cloud_low_intensity->empty()) {
        success = partitionCellACiri(cell_A,
                                     cell_B,
                                     plane_normals,
                                     plane_points,
                                     agent_pos,
                                     waypoint,
                                     neighbors_pos,
                                     cloud_low_intensity,
                                     c1,
                                     seed_b,
                                     threshold_active);
      }
      else {
        std::cout << "[RBLController]: Skipping Ciri, low-intensity cloud is empty." << std::endl;
      }
      if (cell_B == cell_A) {
        std::cout << "[RBLController]: equal" << std::endl;
      } 

      if (!success || cell_A.size() == 0) {
        std::cout << "[RBLController]: Ciri failed. Using classic partition." << std::endl;
        cell_A.clear();
        partitionCellA(cell_A, cell_S, plane_normals, plane_points, agent_pos, neighbors_pos, cloud);
        seed_b = agent_pos;
      }
    }
    else {
      partitionCellA(cell_A, cell_S, plane_normals, plane_points, agent_pos, neighbors_pos, cloud);
    }
  // }
  // else {
  //   cell_A = cell_S;
  // }
  sensed_cell_A = computeActivelySensedCell(cell_A, agent_pos, rpy);
}  // //}

std::vector<Eigen::Vector3d> RBLController::computeActivelySensedCell(std::vector<Eigen::Vector3d>& cell_A,
                                                                      const Eigen::Vector3d&        agent_pos,
                                                                      const Eigen::Vector3d&        rpy)  // //{
{
  std::vector<Eigen::Vector3d> sensed_cell    = cell_A;
  double                       lidar_tilt_rad = params_.lidar_tilt * M_PI / 180.0;
  double                       lidar_fov_rad  = params_.lidar_fov * M_PI / 180.0;
  Eigen::Vector3d              lidar_pos      = agent_pos;
  Eigen::Matrix3d              R_rpy          = Rx(rpy[0]) * Ry(rpy[1]) * Rz(rpy[2]);

  Eigen::Vector3d norm = Ry(lidar_tilt_rad) * Eigen::Vector3d(0, 0, 1);
  norm                 = R_rpy * norm;

  Eigen::Vector3d norm_upper = Ry(lidar_tilt_rad - lidar_fov_rad) * Eigen::Vector3d(0, 0, 1);
  norm_upper                 = R_rpy * norm_upper;

  sensed_cell.erase(std::remove_if(sensed_cell.begin(),
                                   sensed_cell.end(),
                                   [&](const Eigen::Vector3d& point) {
                                     double res       = norm.dot(point - lidar_pos);
                                     double res_upper = norm_upper.dot(point - lidar_pos);
                                     return res < 0 || res_upper > 0;
                                   }),
                    sensed_cell.end());
  return sensed_cell;
}  // //}

Eigen::Matrix3d RBLController::Rx(double angle)  // //{
{
  Eigen::Matrix3d Rx;
  Rx << 1, 0, 0, 0, std::cos(angle), -std::sin(angle), 0, std::sin(angle), std::cos(angle);
  return Rx;
}  // //}

Eigen::Matrix3d RBLController::Ry(double angle)  // //{
{
  Eigen::Matrix3d Ry;
  Ry << std::cos(angle), 0, std::sin(angle), 0, 1, 0, -std::sin(angle), 0, std::cos(angle);
  return Ry;
}  // //}

Eigen::Matrix3d RBLController::Rz(double angle)  // //{
{
  Eigen::Matrix3d Rz;
  Rz << std::cos(angle), -std::sin(angle), 0, std::sin(angle), std::cos(angle), 0, 0, 0, 1;
  return Rz;
}  // //}

Eigen::Vector3d RBLController::movePointToCell(const Eigen::Vector3d&              point,
                                               const std::vector<Eigen::Vector3d>& cell)  // //{
{
  if (cell.empty()) {
    return point;
  }

  Eigen::Vector3d closest_point = cell[0];

  double min_dist_sq = (point - cell[0]).squaredNorm();

  for (size_t i = 1; i < cell.size(); ++i) {
    double current_dist_sq = (point - cell[i]).squaredNorm();

    if (current_dist_sq < min_dist_sq) {
      min_dist_sq   = current_dist_sq;
      closest_point = cell[i];
    }
  }
  return closest_point;
}  // //}

void RBLController::computeCentroid(Eigen::Vector3d&              centroid,  // //{
                                    Eigen::Vector3d&              agent_pos,
                                    Eigen::Vector3d&              agent_vel,
                                    std::vector<Eigen::Vector3d>& cell,
                                    std::vector<Eigen::Vector3d>& plane_normals,
                                    std::vector<Eigen::Vector3d>& plane_points,
                                    Eigen::Vector3d&              destination,
                                    Eigen::Vector3d&              goal,
                                    double&                       beta,
                                    bool                          flag_threshold,
                                    bool&                         threshold_active)
{
  (void)agent_vel;     // TODO - currently unused

  if (cell.empty()) {
    centroid = agent_pos;
    threshold_active = false;
    std::cout << "[RBLController]: computeCentroid received empty cell, using agent position." << std::endl;
    return;
  }

  std::cout << "destitnation" << destination << std::endl;
  std::vector<double> x_in, y_in, z_in;
  for (const auto& point : cell) {
    x_in.push_back(point[0]);
    y_in.push_back(point[1]);
    z_in.push_back(point[2]);
  }

  std::vector<double> scalar_values;
  computeScalarValue(scalar_values, x_in, y_in, z_in, destination, goal, beta);

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
  if (sum <= std::numeric_limits<double>::epsilon() || !std::isfinite(sum)) {
    centroid = agent_pos;
    threshold_active = false;
    std::cout << "[RBLController]: computeCentroid received invalid weights, using agent position." << std::endl;
    return;
  }
  centroid = Eigen::Vector3d(sum_x / sum, sum_y / sum, sum_z / sum);

  double min_distance        = std::numeric_limits<double>::max();
  int closest_plane_index = -1;

  for (size_t i = 0; i < plane_normals.size(); ++i) {
    const Eigen::Vector3d& normal         = plane_normals[i];
    const Eigen::Vector3d& point_on_plane = plane_points[i];

    Eigen::Vector3d normalized_normal = normal.normalized();

    double distance = std::abs((centroid - point_on_plane).dot(normalized_normal));

    if (distance < min_distance) {
      min_distance        = distance;
      closest_plane_index = static_cast<int>(i);
    }
  }

  if (closest_plane_index == -1) {
    min_distance = params_.radius - (agent_pos - centroid).norm();
  }

  std::cout << "[RBLController]: vel: " << agent_vel_.norm() << ", beta: " << beta << ", threshold "<< threshold_active << std::endl;
  // double dist_centroid_to_boundary = std::sqrt(std::pow((centroid[0] - ), 2) + std::pow((centroid[1] - ), 2) +
  // std::pow((centroid[2] - ), 2));
 
  if (min_distance < params_.boundary_threshold && beta < 20.0 && agent_vel_.norm() > params_.boundary_threshold_speed) {
    beta = beta + 0.1;
    threshold_active = true;

    // std::cout << "[RBLController]: computing centroid again. new beta: " << beta << ", distance to boundary: " <<
    // min_distance << std::endl;
    computeCentroid(centroid,
                    agent_pos,
                    agent_vel_,
                    cell,
                    plane_normals,
                    plane_points,
                    destination,
                    waypoint_fixed_distance_,
                    beta,
                    flag_threshold,
                    threshold_active);
  }else{
   threshold_active = false; 
  }
  // if (beta > 1.5) {
  //   centroid = agent_pos;
  // }

  // if (agent_vel_.norm() > params_.boundary_threshold_speed) {
  //   threshold_active = true;
  // }else{
  //   threshold_active = false; 
  // }

}  // //}

void RBLController::computeScalarValue(std::vector<double>&       scalar_values,  // //{
                                       const std::vector<double>& x_test,
                                       const std::vector<double>& y_test,
                                       const std::vector<double>& z_test,
                                       const Eigen::Vector3d&     destination,
                                       const Eigen::Vector3d&     goal,
                                       double                     beta)
{
  (void)goal;     // TODO - currently unused

  for (size_t i = 0; i < x_test.size(); ++i) {
    double distance = std::sqrt(std::pow((x_test[i] - destination[0]), 2) + std::pow((y_test[i] - destination[1]), 2) +
                                std::pow((z_test[i] - destination[2]), 2));

    // double distance_to_goal= std::sqrt(std::pow((x_test[i] - goal[0]), 2) + std::pow((y_test[i] - goal[1]), 2) +
    //                             std::pow((z_test[i] - goal[2]), 2));

    double scalar_value = std::exp(-distance / beta);
    scalar_values.push_back(scalar_value);
  }
}  // //}

void RBLController::applyRules(double&                beta,  // //{
                               double&                th,
                               double&                ph,
                               Eigen::Vector3d        destination,
                               Eigen::Vector3d&       seed_b_,
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
  (void)seed_b_;     // TODO - currently unused

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
      // seed_b_ = goal;
      // seed_b_ = seed_b_ - 2 * dt * (seed_b_ - (agent_pos + 0.5*(destination-agent_pos)/(destination-agent_pos).norm()));
      // seed_b_ =  c1;
    }
    else {
      beta = std::max(beta - dt * (beta - betaD), beta_min);
      // seed_b_ = agent_pos;
      // seed_b_ = seed_b_ -  dt * (seed_b_ - agent_pos);
    }
    // seed_b_ = c1;
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

    // th = 0;


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
      if (((dist_c1_c2_z > d5) && (dist_current_c1_z < d6)) ||
          ((dist_current_c2_plane_xy - dist_current_c1_plane_xy) > d7)) {  // modify phi up down based on some if
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
    // std::cout << "theta: " << th << ", beta: " << beta << std::endl;

    double dx    = goal[0] - current_j_x;
    double dy    = goal[1] - current_j_y;
    double dz    = goal[2] - current_j_z;
    double dist  = sqrt(dx * dx + dy * dy + dz * dz);
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
      // seed_b_ = seed_b_ - 2 * dt * (seed_b_ - goal);
      // seed_b_ = goal;
    }
    else {
      beta = beta - dt * (beta - betaD);
      // seed_b_ = seed_b_ - 2 * dt * (seed_b_ - agent_pos);
      // seed_b_ = agent_pos;
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

    // std::cout << "theta: " << th << ", beta: " << beta << std::endl;
  }
}  // //}

Eigen::Vector3d RBLController::determineWaypointFixedDistance(const std::vector<Eigen::Vector3d>& path,  // //{
                                                              const Eigen::Vector3d&              agent_pos,
                                                              const Eigen::Vector3d&              goal)
{
  double min_dist_sq         = std::numeric_limits<double>::max();
  size_t    closest_point_index = 0;
  bool found = false;

  // Find closest path point
  for (size_t i = 0; i < path.size(); ++i) {
    double dist_sq = (path[i] - agent_pos).squaredNorm();
    if (dist_sq < min_dist_sq) {
      min_dist_sq         = dist_sq;
      closest_point_index = i;
      found = true;
    }
  }

  // If we're at the end, return the goal
  if (!found || closest_point_index + 1 >= path.size()) {
    return path.back();
  }

  // Circle radius
  double r = params_.radius;

  // Iterate over path segments starting from the closest
  for (size_t i = closest_point_index; i < path.size() - 1; ++i) {
    Eigen::Vector3d p1 = path[i];
    Eigen::Vector3d p2 = path[i + 1];
    Eigen::Vector3d d  = p2 - p1;  // segment direction

    // Quadratic equation for intersection of segment with circle
    Eigen::Vector3d f = p1 - agent_pos;
    double          a = d.dot(d);
    double          b = 2 * f.dot(d);
    double          c = f.dot(f) - r * r;

    double discriminant = b * b - 4 * a * c;
    if (discriminant < 0) {
      continue;  // no intersection
    }

    discriminant = std::sqrt(discriminant);
    double t1    = (-b - discriminant) / (2 * a);
    double t2    = (-b + discriminant) / (2 * a);

    // Check if intersections are within the segment
    if (t1 >= 0.0 && t1 <= 1.0) {
      return p1 + t1 * d;
    }
    if (t2 >= 0.0 && t2 <= 1.0) {
      return p1 + t2 * d;
    }
  }

  // If no intersection found, fall back to goal
  return goal;
}  // //}

Eigen::Vector3d RBLController::determineWaypoint(const std::vector<Eigen::Vector3d>& path,  // //{
                                                 const Eigen::Vector3d&              agent_pos,
                                                 const Eigen::Vector3d&              goal,
                                                 Eigen::Vector3d&                    waypoint)
{
  double min_dist_sq         = std::numeric_limits<double>::max();
  size_t closest_point_index = -1;
  bool found = false;

  for (size_t i = 0; i < path.size(); ++i) {
    double dist_sq = (path[i] - agent_pos).squaredNorm();
    if (dist_sq < min_dist_sq) {
      min_dist_sq         = dist_sq;
      closest_point_index = i;
      found = true;
    }
  }

  if (!found || closest_point_index + 1 >= path.size()) {
    return path.back();
  }

  double          dist_agent_goal = (goal - agent_pos).norm();
  // Eigen::Vector3d closest_point   = path[closest_point_index];
  Eigen::Vector3d next_point      = path[closest_point_index + 1];
  // Eigen::Vector3d direction_vector = (next_point - closest_point) / (next_point - closest_point).norm();
  Eigen::Vector3d direction_vector      = (next_point - agent_pos) / (next_point - agent_pos).norm();
  double          dist_agent_next_point = (next_point - agent_pos).norm();
  // std::cout << "[RBLController]: Distance agent to next point on path: " << dist_agent_next_point << std::endl;
  // Eigen::Vector3d waypoint;
  // if (params_.ciri) {
  //   waypoint = next_point;
  // } else {
  //   waypoint = closest_point + direction_vector * std::min(params_.radius, dist_agent_goal);
  // }
  // waypoint = closest_point + a * std::min(params_.radius, dist_agent_goal);

  // waypoint = agent_pos + direction_vector * std::min(params_.radius, dist_agent_next_point);
  double tmp_min = std::min(params_.radius, dist_agent_next_point);
  if (dist_agent_goal > params_.radius) {
    // waypoint = agent_pos + direction_vector * std::max(tmp_min,params_.radius);
    waypoint = waypoint + 2 * params_.dt *
                              (agent_pos + direction_vector * std::max(tmp_min, params_.radius / 2) - waypoint) /
                              (agent_pos + direction_vector * std::max(tmp_min, params_.radius / 2) - waypoint).norm();
  }
  else {
    // waypoint = agent_pos + direction_vector * std::max(tmp_min, dist_agent_goal);
    // waypoint = agent_pos + direction_vector * std::min(params_.radius, dist_agent_next_point);
    waypoint = waypoint +
               2 * params_.dt *
                   (agent_pos + direction_vector * std::min(params_.radius, dist_agent_next_point) - waypoint) /
                   (agent_pos + direction_vector * std::min(params_.radius, dist_agent_next_point) - waypoint).norm();
  }

  return waypoint;
  // return next_point;
}  // //}

void RBLController::determineNextRef(mrs_msgs::msg::Reference&                p_ref,  // //{
                                     const Eigen::Vector3d&              agent_pos,
                                     const Eigen::Vector3d&              waypoint,
                                     const Eigen::Vector3d&              goal,
                                     const Eigen::Vector3d&              c1,
                                     const Eigen::Vector3d&              c1_full,
                                     const Eigen::Vector3d&              rpy,
                                     const std::vector<Eigen::Vector3d>& path)
{
  (void)waypoint;     // TODO - currently unused
  (void)path;     // TODO - currently unused

  if (params_.limited_fov) {
    double desired_heading;
    // if (params_.replanner) {
    //   desired_heading = determineYaw(agent_pos, waypoint, path, rpy);
    // } else {
    // desired_heading = std::atan2(c1_full[1] - agent_pos[1], c1_full[0] - agent_pos[0]);
    // }

    double heading_to_centroid = std::atan2(c1_full[1] - agent_pos[1], c1_full[0] - agent_pos[0]);
    double diff                = std::fmod(heading_to_centroid - rpy[2] + M_PI, 2 * M_PI) - M_PI;
    double difference          = (diff < -M_PI) ? diff + 2 * M_PI : diff;

    if (std::abs(difference) < M_PI / 2) {  //+-90 deg
      p_ref.position.x = c1[0];
      p_ref.position.y = c1[1];
      p_ref.position.z = c1[2];
    }
    else {
      p_ref.position.x = agent_pos[0];
      p_ref.position.y = agent_pos[1];
      p_ref.position.z = agent_pos[2];
    }

    // if ((c1 - c1_full).norm() < 0.5) {
    //   desired_heading = std::atan2(c1[1] - agent_pos[1], c1[0] - agent_pos[0]);
    // }
    // else {
      desired_heading = std::atan2(c1_full[1] - agent_pos[1], c1_full[0] - agent_pos[0]);
    // }
    p_ref.heading = desired_heading;

    if ((agent_pos - goal).norm() <= 0.3) {  // Arived at goal pos
      // p_ref.heading = rpy[2]; //keep the same heading
      p_ref.heading = desired_heading;
      std::cout << "[RBLController]: Arrived at goal pos" << std::endl;  // keep the same heading
    }
    else {
      p_ref.heading = desired_heading;
    }
  }
  else {
    p_ref.position.x = c1[0];
    p_ref.position.y = c1[1];
    p_ref.position.z = c1[2];
  }
}  // //}

mrs_msgs::msg::Reference RBLController::pRefAgent(const Eigen::Vector3d& agent_pos,
                                             const double           yaw)  // //{
{
  mrs_msgs::msg::Reference p_ref;
  p_ref.position.x = agent_pos.x();
  p_ref.position.y = agent_pos.y();
  p_ref.position.z = agent_pos.z();
  p_ref.heading    = yaw;
  return p_ref;
}  // //}

double RBLController::determineYaw(const Eigen::Vector3d&              agent_pos,
                                   const Eigen::Vector3d&              waypoint,
                                   const std::vector<Eigen::Vector3d>& path,
                                   const Eigen::Vector3d&              rpy)  // //{
{
  Eigen::Vector3d direction_vector = waypoint - agent_pos;
  double          yaw_to_waypoint  = std::atan2(direction_vector.y(), direction_vector.x());

  auto it = std::find(path.begin(), path.end(), waypoint);
  if (it == path.end() || std::next(it) == path.end()) {
    return yaw_to_waypoint;
  }

  size_t waypoint_idx = std::distance(path.begin(), it);

  Eigen::Vector3d direction_to_next    = path[waypoint_idx + 1] - waypoint;
  double          yaw_to_next_waypoint = std::atan2(direction_to_next.y(), direction_to_next.x());

  double distance_to_waypoint = direction_vector.norm();
  double dis_interpolation    = 3.0;  // [m]

  if (distance_to_waypoint > dis_interpolation) {
    return yaw_to_waypoint;
  }
  else {
    double current_yaw           = rpy.z();
    double delta_yaw_to_waypoint = yaw_to_waypoint - current_yaw;
    double delta_yaw_to_next     = yaw_to_next_waypoint - current_yaw;

    while (delta_yaw_to_waypoint > M_PI)
      delta_yaw_to_waypoint -= 2 * M_PI;
    while (delta_yaw_to_waypoint < -M_PI)
      delta_yaw_to_waypoint += 2 * M_PI;

    while (delta_yaw_to_next > M_PI)
      delta_yaw_to_next -= 2 * M_PI;
    while (delta_yaw_to_next < -M_PI)
      delta_yaw_to_next += 2 * M_PI;

    double t                      = dis_interpolation - distance_to_waypoint;
    double interpolated_delta_yaw = (1 - t) * delta_yaw_to_waypoint + t * delta_yaw_to_next;

    double max_yaw_change = M_PI / 4.0;  // 45 degrees
    if (interpolated_delta_yaw > max_yaw_change) {
      interpolated_delta_yaw = max_yaw_change;
    }
    else if (interpolated_delta_yaw < -max_yaw_change) {
      interpolated_delta_yaw = -max_yaw_change;
    }
    return current_yaw + interpolated_delta_yaw;
  }
}  // //}

double RBLController::normalizeAngle(double angle)  // //{
{
  if (angle > M_PI) {
    angle -= 2 * M_PI;
  }
  else if (angle < -M_PI) {
    angle += 2 * M_PI;
  }
  return angle;
}  // //}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>
RBLController::downSamplePcl(std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud,  // //{
                             double                                           voxel_size)
{
  // std::cout << "[RBLController]: Voxelizing PointCloud to voxel size: " << voxel_size << std::endl; //TODO uncomment

  pcl::PointCloud<pcl::PointXYZI>::Ptr boost_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*cloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr boost_voxelized_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud(boost_cloud);
  sor.setLeafSize(static_cast<float>(voxel_size), static_cast<float>(voxel_size), static_cast<float>(voxel_size));
  sor.filter(*boost_voxelized_cloud);

  return std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>(boost_voxelized_cloud.get(), [boost_voxelized_cloud](...) {});
}  // //}
