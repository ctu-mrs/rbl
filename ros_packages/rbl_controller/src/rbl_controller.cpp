#include <rbl_controller.h>

namespace formation_control
{
// Get the start time
/*RBLController::onInit () //{ */
void RBLController::onInit() {
  // initialize nodelet
  ros::NodeHandle &nh = getPrivateNodeHandle();
  NODELET_DEBUG("Initializing nodelet...");
  ros::Time::waitForValid();

  // load parameters
  std::string          _leader_name;
  std::vector<int>     _uvdar_ids_;
  mrs_lib::ParamLoader param_loader(nh, "RBLController");

  /*RBLController::LoadPram //{ */
  param_loader.loadParam("uav_names", _uav_names_);
  param_loader.loadParam("uav_uvdar_ids", _uvdar_ids_);
  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("odometry_topic", _odometry_topic_name_);
  param_loader.loadParam("set_reference_timer/rate", _rate_timer_set_reference_);
  param_loader.loadParam("control_frame", _control_frame_);
  param_loader.loadParam("odom_msg_max_latency", _odom_msg_max_latency_);
  param_loader.loadParam("diagnostics/odom_timeout", _odom_timeout_);
  param_loader.loadParam("diagnostics/rate", _rate_timer_diagnostics_);
  param_loader.loadParam("d1", d1);
  param_loader.loadParam("d2", d2);
  param_loader.loadParam("d3", d3);
  param_loader.loadParam("d4", d4);
  param_loader.loadParam("radius", radius);
  param_loader.loadParam("encumbrance", encumbrance);
  param_loader.loadParam("step_size", step_size);
  param_loader.loadParam("betaD", betaD);
  param_loader.loadParam("beta_min", beta_min);
  param_loader.loadParam("dt", dt);
  param_loader.loadParam("initial_positions/" + _uav_name_ + "/x", _required_initial_position_[0]);
  param_loader.loadParam("initial_positions/" + _uav_name_ + "/y", _required_initial_position_[1]);
  param_loader.loadParam("initial_positions/" + _uav_name_ + "/z", _required_initial_position_[2]);
  param_loader.loadParam("monitored_area_origin/x", _monitored_area_origin_[0]);
  param_loader.loadParam("monitored_area_origin/y", _monitored_area_origin_[1]);
  param_loader.loadParam("monitored_area_origin/z", _monitored_area_origin_[2]);
  param_loader.loadParam("max_distance_to_initial_position", _dist_to_start_limit_);
  param_loader.loadParam("maximum_distance_conn", maximum_distance_conn);
  param_loader.loadParam("size_neighbors1", size_neighbors1);
  param_loader.loadParam("size_obstacles1", size_obstacles1);
  param_loader.loadParam("final_positions/" + _uav_name_ + "/x", destination.first);
  param_loader.loadParam("final_positions/" + _uav_name_ + "/y", destination.second);
  param_loader.loadParam("Adj_matrix_" + _uav_name_, Adj_matrix);
  param_loader.loadParam("noisy_measurements", noisy_measurements);
  param_loader.loadParam("noisy_angle", noisy_angle);
  param_loader.loadParam("threshold", threshold);
  param_loader.loadParam("window_length", window_length);
  param_loader.loadParam("bias_error", bias_error);
  param_loader.loadParam("cwvd_rob", cwvd_rob);
  param_loader.loadParam("cwvd_obs", cwvd_obs);
  param_loader.loadParam("refZ", refZ_);
  param_loader.loadParam("simulation", simulation_);
  param_loader.loadParam("replanner_flag", replanner_flag);
  param_loader.loadParam("connectivity_flag", connectivity_flag);

  double tmp;
  param_loader.loadParam("max_obstacle_integration_dist", tmp);
  max_obstacle_integration_dist_sqr_ = pow(tmp, 2);

  // param_loader.loadParam("initial_positions/" + _uav_name_ + "/z", destination[2]);
  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[RblController]: Could not load all parameters!");
    ros::shutdown();
  }
  //}

  _required_initial_position_ += _monitored_area_origin_;
  destination.first += _monitored_area_origin_[0];
  destination.second += _monitored_area_origin_[1];
  size_neighbors.assign(_uav_names_.size() - 1, size_neighbors1);

  // erase this uav from the list of uavs
  auto it = std::find(_uav_names_.begin(), _uav_names_.end(), _uav_name_);

  if (it != _uav_names_.end()) {
    this_uav_idx_ = it - _uav_names_.begin();
    _uav_names_.erase(it);
    _uvdar_ids_.erase(_uvdar_ids_.begin() + this_uav_idx_);
  } else {
    ROS_ERROR("[RBLController]: This UAV is not part of the formation! Check the config file. Shutting down node.");
    ros::shutdown();
  }
  beta      = betaD;
  n_drones_ = _uav_names_.size();

  uav_neighbors_.resize(n_drones_);
  uav_positions_.resize(n_drones_);
  neighbors_and_obstacles_noisy.resize(n_drones_);

  goal[0]          = destination.first;
  goal[1]          = destination.second;
  goal_original[0] = destination.first;
  goal_original[1] = destination.second;

  // std::vector<Eigen::Vector3d> uav_positionsN_(uav_positions_);
  /* create multiple subscribers to read uav odometries */
  // iterate through drones except this drone and target
  for (int i = 0; i < _uav_names_.size(); i++) {
     // std::string topic_name = std::string("/") + _uav_names_[i] + std::string("/") + _odometry_topic_name_; 
     // other_uav_odom_subscribers_.push_back(nh.subscribe<nav_msgs::Odometry>(topic_name.c_str(), 1, boost::bind(&RBLController::odomCallback, this, _1, i)));
     _uav_uvdar_ids_[_uvdar_ids_[i]] = i;
    /*   ROS_INFO("Subscribing to %s", topic_name.c_str()); */
  }

  uav_odom_subscriber_ = nh.subscribe("/" + _uav_name_ + "/estimation_manager/odom_main", 1, &RBLController::odomCallback, this);
  sub_uvdar_filtered_poses_.push_back(nh.subscribe<mrs_msgs::PoseWithCovarianceArrayStamped>(
      "/" + _uav_name_ + "/uvdar/measuredPoses", 1, boost::bind(&RBLController::callbackNeighborsUsingUVDAR, this, _1)));
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = "RBLController";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_position_command_ = mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>(shopts, "tracker_cmd_in");
  clusters_sub_.push_back(nh.subscribe<visualization_msgs::MarkerArray>("/" + _uav_name_ + "/rplidar/clusters_", 1, &RBLController::clustersCallback, this));

  clusters_sub_1.push_back(nh.subscribe<visualization_msgs::MarkerArray>("/" + _uav_name_ + "/rplidar/clusters_1", 1, &RBLController::clustersCallback1, this));

  waypoints_sub_.push_back(
      nh.subscribe<visualization_msgs::MarkerArray>("/" + _uav_name_ + "/replanner/waypoints_", 1, &RBLController::waypointsCallback, this));

  // initialize timers
  timer_set_reference_ = nh.createTimer(ros::Rate(_rate_timer_set_reference_), &RBLController::callbackTimerSetReference, this);
  timer_diagnostics_   = nh.createTimer(ros::Rate(_rate_timer_diagnostics_), &RBLController::callbackTimerDiagnostics, this);
  // timer_pub_ = nh.createTimer(ros::Rate(_rate_timer_set_reference_), &RBLController::callbackPublisher, this);

  // initialize service servers
  service_activate_control_   = nh.advertiseService("control_activation_in", &RBLController::activationServiceCallback, this);
  service_deactivate_control_ = nh.advertiseService("control_deactivation_in", &RBLController::deactivationServiceCallback, this);
  service_activate_params_control_   = nh.advertiseService("control_activation_params_in", &RBLController::activationParamsServiceCallback, this);
  service_fly_to_start_       = nh.advertiseService("fly_to_start_in", &RBLController::flyToStartServiceCallback, this);

  // initialize service clients
  sc_set_position_  = nh.serviceClient<mrs_msgs::ReferenceStampedSrv>("ref_pos_out");
  sc_goto_position_ = nh.serviceClient<mrs_msgs::Vec4>("goto_out");

  // initialize publishers
  pub_obstacles_ = nh.advertise<visualization_msgs::MarkerArray>("obstacle_markers_out", 1, true);

  pub_neighbors_   = nh.advertise<visualization_msgs::MarkerArray>("neighbors_markers_out", 1, true);
  pub_destination_ = nh.advertise<visualization_msgs::Marker>("destination_out", 1, true);
  pub_position_    = nh.advertise<visualization_msgs::Marker>("position_out", 1, true);
  pub_line_        = nh.advertise<visualization_msgs::Marker>("line_out", 1, true);
  pub_hull_        = nh.advertise<visualization_msgs::MarkerArray>("hull_markers_out", 1, true);
  pub_centroid_    = nh.advertise<visualization_msgs::Marker>("centroid_out", 1, true);
  // initialize transformer
  transformer_ = std::make_shared<mrs_lib::Transformer>(nh, "RBLController");
  /* transformer_->setDefaultPrefix(_uav_name_); */
  transformer_->retryLookupNewest(true);

  is_initialized_ = true;
  ROS_INFO("[RBLController]: Initialization completed.");
}
//}

/*Publisher for Visualization //{ */

/*RBLController::publishObstacles () //{ */
void RBLController::publishObstacles() {

  visualization_msgs::MarkerArray obstacle_markers;
  {
    std::scoped_lock lock(mutex_obstacles_);
    size_t           current_obstacle_count = obstacles_.size();

    // Add or update markers for existing obstacles
    for (size_t i = 0; i < current_obstacle_count; ++i) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = _control_frame_;
      marker.header.stamp    = ros::Time::now();
      marker.ns              = "obstacles";
      marker.id              = i;
      marker.type            = visualization_msgs::Marker::CYLINDER;
      marker.action          = visualization_msgs::Marker::ADD;

      marker.pose.position.x    = obstacles_[i].first;
      marker.pose.position.y    = obstacles_[i].second;
      marker.pose.position.z    = 0;  // Assuming obstacles are on the ground
      marker.pose.orientation.w = 1.0;
      marker.scale.x            = 2 * size_obstacles1;  // Adjust size as necessary
      marker.scale.y            = 2 * size_obstacles1;
      marker.scale.z            = 5.0;
      marker.color.r            = 1.0;  // Red color
      marker.color.g            = 0.0;
      marker.color.b            = 0.0;
      marker.color.a            = 1.0;  // Fully opaque

      obstacle_markers.markers.push_back(marker);
    }
  }

  // Publish the marker array

  pub_obstacles_.publish(obstacle_markers);
}
//}

/*RBLController::publishNeighbors () //{ */
void RBLController::publishNeighbors() {
  visualization_msgs::MarkerArray neighbors_markers;


  for (size_t i = 0; i < neighbors_and_obstacles_noisy.size(); ++i) {
    visualization_msgs::Marker marker;
    marker.header.frame_id    = _control_frame_;
    marker.header.stamp       = ros::Time::now();
    marker.ns                 = "neighbors_and_obstacles_noisy";
    marker.id                 = i;
    marker.type               = visualization_msgs::Marker::CYLINDER;
    marker.action             = visualization_msgs::Marker::ADD;
    marker.pose.position.x    = neighbors_and_obstacles_noisy[i].first;
    marker.pose.position.y    = neighbors_and_obstacles_noisy[i].second;
    marker.pose.position.z    = 0;  // Assuming obstacles are on the ground
    marker.pose.orientation.w = 1.0;
    marker.scale.x            = 0.3;  // Adjust size as necessary
    marker.scale.y            = 0.3;
    marker.scale.z            = 5.0;
    marker.color.r            = 0.0;  // Red color
    marker.color.g            = 1.0;
    marker.color.b            = 0.0;
    marker.color.a            = 1.0;  // Fully opaque
    neighbors_markers.markers.push_back(marker);
  }

  pub_neighbors_.publish(neighbors_markers);
}

//}

/*RBLController::publishHull() //{ */
void RBLController::publishHull() {
  visualization_msgs::MarkerArray hull_voro_markers;

  for (size_t i = 0; i < hull_voro.size(); ++i) {
    visualization_msgs::Marker marker;
    marker.header.frame_id    = _control_frame_;
    marker.header.stamp       = ros::Time::now();
    marker.ns                 = "hull_voro";
    marker.id                 = i;
    marker.type               = visualization_msgs::Marker::CYLINDER;
    marker.action             = visualization_msgs::Marker::ADD;
    marker.pose.position.x    = hull_voro[i].first;
    marker.pose.position.y    = hull_voro[i].second;
    marker.pose.position.z    = 0;  // Assuming obstacles are on the ground
    marker.pose.orientation.w = 1.0;
    marker.scale.x            = 0.1;  // Adjust size as necessary
    marker.scale.y            = 0.1;
    marker.scale.z            = 5.0;
    marker.color.r            = 0.0;  // Red color
    marker.color.g            = 1.0;
    marker.color.b            = 0.0;
    marker.color.a            = 1.0;  // Fully opaque
    hull_voro_markers.markers.push_back(marker);
  }

  pub_hull_.publish(hull_voro_markers);
}
//}

/*RBLController::publishPosition() //{ */
void RBLController::publishPosition() {
  visualization_msgs::Marker marker;
  marker.header.frame_id    = _control_frame_;
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "position";
  marker.id                 = 0;
  marker.type               = visualization_msgs::Marker::CYLINDER;
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.position.x    = robot_pos.first;
  marker.pose.position.y    = robot_pos.second;
  marker.pose.position.z    = 0;  // Assuming obstacles are on the ground
  marker.pose.orientation.w = 1.0;
  marker.scale.x            = 2 * encumbrance;  // Adjust size as necessary
  marker.scale.y            = 2 * encumbrance;
  marker.scale.z            = 1.0;
  marker.color.r            = 0.0;  // Red color
  marker.color.g            = 0.0;
  marker.color.b            = 1.0;
  marker.color.a            = 1.0;  // Fully opaque

  pub_position_.publish(marker);
}
//}


void RBLController::publishLine() {
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = _control_frame_;
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "neighbors";
    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.action = visualization_msgs::Marker::ADD;

    // Line thickness
    line_list.scale.x = 0.05;

    // Line color (blue)
    line_list.color.r = 0.0f;
    line_list.color.g = 1.0f;
    line_list.color.b = 0.0f;
    line_list.color.a = 0.5f;

    // Pose (must be valid!)
    line_list.pose.position.x = 0.0;
    line_list.pose.position.y = 0.0;
    line_list.pose.position.z = 0.0;
    line_list.pose.orientation.x = 0.0;
    line_list.pose.orientation.y = 0.0;
    line_list.pose.orientation.z = 0.0;
    line_list.pose.orientation.w = 1.0;  // identity quaternion

    // Build the line list
    line_list.points.clear();

    geometry_msgs::Point robot_point;
    robot_point.x = robot_pos.first;
    robot_point.y = robot_pos.second;
    robot_point.z = 0.0;

    for (auto& uav : fixed_neighbors_vec) {
        geometry_msgs::Point uav_point;
        uav_point.x = uav.first;
        uav_point.y = uav.second;
        uav_point.z = 0.0;

        // LINE_LIST expects pairs of points
        line_list.points.push_back(robot_point);  // start
        line_list.points.push_back(uav_point);    // end
    }

    // Publish marker
    pub_line_.publish(line_list);
}

/*RBLController::publishCentroid() //{ */
void RBLController::publishCentroid() {
  visualization_msgs::Marker marker;
  marker.header.frame_id    = _control_frame_;
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "centroid";
  marker.id                 = 0;
  marker.type               = visualization_msgs::Marker::CYLINDER;
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.position.x    = 0;
  marker.pose.position.y    = 1;
  marker.pose.position.z    = 0;  // Assuming obstacles are on the ground
  marker.pose.orientation.w = 1.0;
  marker.scale.x            = 0.1;  // Adjust size as necessary
  marker.scale.y            = 0.1;
  marker.scale.z            = 1.0;
  marker.color.r            = 1.0;  // Red color
  marker.color.g            = 0.0;
  marker.color.b            = 0.0;
  marker.color.a            = 1.0;  // Fully opaque

  pub_centroid_.publish(marker);
}
//}

/*RBLController::publishDestination() //{ */
void RBLController::publishDestination() {
  visualization_msgs::Marker marker;
  marker.header.frame_id    = _control_frame_;
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "destination";
  marker.id                 = 0;
  marker.type               = visualization_msgs::Marker::CYLINDER;
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.position.x    = goal[0];
  marker.pose.position.y    = goal[1];
  marker.pose.position.z    = 0;  // Assuming obstacles are on the ground
  marker.pose.orientation.w = 1.0;
  marker.scale.x            = 1.0;  // Adjust size as necessary
  marker.scale.y            = 1.0;
  marker.scale.z            = 1.0;
  marker.color.r            = 0.0;  // Red color
  marker.color.g            = 0.5;
  marker.color.b            = 0.5;
  marker.color.a            = 0.5;  // Fully opaque

  pub_destination_.publish(marker);
}
//}
//}

/*RBLController::functions() //{ */

typedef std::pair<double, double> Point;

// Function to find the closest waypoint at a distance near 4 meters
std::pair<double, double> RBLController::getClosestWaypoint(const std::pair<double, double> &robot_pos, double target_distance = 4.0) {
  double                    closest_distance = std::numeric_limits<double>::infinity();  // Set to a large number
  std::pair<double, double> closest_waypoint;

  for (const auto &waypoint : waypoints_) {
    double distance = calculateDistance(robot_pos, waypoint);

    // If the distance is closer to the target (4 meters), update closest_distance
    if (std::abs(distance - target_distance) < std::abs(closest_distance - target_distance)) {
      closest_distance = distance;
      closest_waypoint = waypoint;
    }
  }

  // If no waypoint is found, return an empty pair (indicating no valid waypoint found)
  if (closest_distance == std::numeric_limits<double>::infinity()) {
    ROS_WARN("No waypoints available.");
    return std::make_pair(0.0, 0.0);
  }

  ROS_INFO_STREAM("Closest waypoint found: (" << closest_waypoint.first << ", " << closest_waypoint.second << ")");
  return closest_waypoint;
}

// Function to calculate the Euclidean distance between two points
double RBLController::calculateDistance(const std::pair<double, double> &p1, const std::pair<double, double> &p2) {
  return std::sqrt(std::pow(p2.first - p1.first, 2) + std::pow(p2.second - p1.second, 2));
}


double RBLController::cross(const Point &O, const Point &A, const Point &B) {
  return (A.first - O.first) * (B.second - O.second) - (A.second - O.second) * (B.first - O.first);
}

/* double RBLController::getDistToInitialPosition() { */
/*   getPositionCmd(); */
/*   double dist = sqrt(pow(position_command_.x - _required_initial_position_[0], 2) + pow(position_command_.y - _required_initial_position_[1], 2) + */
/*                      pow(position_command_.z - _required_initial_position_[2], 2)); */
/*   return dist; */
/* } */

double RBLController::euclideanDistance(const Point &a, const Point &b) {
  double dx = a.first - b.first;
  double dy = a.second - b.second;
  return sqrt(dx * dx + dy * dy);
}

std::vector<Point> RBLController::convexHull(std::vector<Point> points) {
  int n = points.size(), k = 0;
  if (n <= 3)
    return points;  // A set of 3 or fewer points is already a convex set

  std::vector<Point> hull(2 * n);

  // Sort points lexicographically (first by x coordinate, then by y coordinate)
  sort(points.begin(), points.end());

  // Build the lower hull
  for (int i = 0; i < n; ++i) {
    while (k >= 2 && cross(hull[k - 2], hull[k - 1], points[i]) <= 0)
      k--;
    hull[k++] = points[i];
  }

  // Build the upper hull
  for (int i = n - 1, t = k + 1; i > 0; --i) {
    while (k >= t && cross(hull[k - 2], hull[k - 1], points[i - 1]) <= 0)
      k--;
    hull[k++] = points[i - 1];
  }

  hull.resize(k - 1);  // Remove the last point because it is repeated at the beginning of the upper hull
  return hull;
}

std::vector<std::pair<double, double>> RBLController::points_inside_circle(std::pair<double, double> robot_pos, double radius, double step_size) {
  double x_center = robot_pos.first;
  double y_center = robot_pos.second;
  int    x_min    = static_cast<int>((x_center - radius) / step_size);
  int    x_max    = static_cast<int>((x_center + radius) / step_size);
  int    y_min    = static_cast<int>((y_center - radius) / step_size);
  int    y_max    = static_cast<int>((y_center + radius) / step_size);

  std::vector<double> x_coords, y_coords;
  for (int i = x_min; i <= x_max; ++i)
    x_coords.push_back(i * step_size);
  for (int j = y_min; j <= y_max; ++j)
    y_coords.push_back(j * step_size);

  std::vector<std::pair<double, double>> points;
  for (auto x : x_coords) {
    for (auto y : y_coords) {
      double distance = std::sqrt(std::pow((x - x_center), 2) + std::pow((y - y_center), 2));
      if (distance <= radius)
        points.push_back(std::make_pair(x, y));
    }
  }
  return points;
}

std::vector<std::pair<double, double>> RBLController::insert_pair_at_index(const std::vector<std::pair<double, double>> &vec, size_t idx,
                                                                           const std::pair<double, double> &value) {
  std::vector<std::pair<double, double>> new_vec = vec;
  if (idx >= new_vec.size()) {
    // If the index is out of bounds, push the value to the back of the vector
    new_vec.push_back(value);
  } else {
    // Insert the value at the specified index
    new_vec.insert(new_vec.begin() + idx, value);
  }
  return new_vec;
}

std::vector<std::pair<double, double>> RBLController::fixed_neighbors(const std::vector<std::pair<double, double>> &positions,
                                                                      const std::vector<int> &adjacency_matrix, size_t my_index) {
  std::vector<std::pair<double, double>> neighbors_filtered;

  // Iterate over the row of the adjacency matrix corresponding to my_index
  for (size_t j = 0; j < adjacency_matrix.size(); ++j) {
    // Check if the value is 1, indicating a neighbor
    if (adjacency_matrix[j] == 1) {
      // Add the position of the neighbor to neighbors_filtered
      neighbors_filtered.push_back(positions[j]);
    }
  }
  return neighbors_filtered;
}

std::vector<std::pair<double, double>> RBLController::communication_constraint(const std::vector<std::pair<double, double>> &points,
                                                                               const std::vector<std::pair<double, double>> &neighbors) {


  new_neighbors.clear();
  // Initialize neighbors_past_measurements if empty
  if (neighbors_past_measurements.size() < neighbors.size()) {
    neighbors_past_measurements.resize(neighbors.size());
  }

  for (size_t j = 0; j < neighbors.size(); ++j) {
    const auto &neighbor = neighbors[j];

    // Add current neighbor to past measurements
    if (neighbors_past_measurements[j].size() >= 10) {
      neighbors_past_measurements[j].erase(neighbors_past_measurements[j].begin());
    }
    neighbors_past_measurements[j].push_back(neighbor);

    // Calculate mean of past measurements
    double mean_x = std::accumulate(neighbors_past_measurements[j].begin(), neighbors_past_measurements[j].end(), 0.0,
                                    [](double sum, const std::pair<double, double> &p) { return sum + p.first; }) /
                    neighbors_past_measurements[j].size();
    double mean_y = std::accumulate(neighbors_past_measurements[j].begin(), neighbors_past_measurements[j].end(), 0.0,
                                    [](double sum, const std::pair<double, double> &p) { return sum + p.second; }) /
                    neighbors_past_measurements[j].size();

    // Check distance from robot_pos to neighbor mean position
    double dx            = mean_x - robot_pos.first;
    double dy            = mean_y - robot_pos.second;
    double distance_1    = std::sqrt(dx * dx + dy * dy);
    double bearing_angle = std::atan2(dy, dx);
    // if (distance_1 > maximum_distance_conn) {
    //   // Project the neighbor position in the same direction but at maximum_distance_conn
    //   mean_x = robot_pos.first + (maximum_distance_conn - threshold) * std::cos(bearing_angle);
    //   mean_y = robot_pos.second + (maximum_distance_conn - threshold) * std::sin(bearing_angle);
    //   /* std::cout << "distance!!!!! :  " << distance_1 << "meanX: " << mean_x << " mean_y: " << mean_y << std::endl; */
    // }

    new_neighbors.push_back({mean_x, mean_y});
  }


  std::vector<int>                       index;
  std::vector<std::pair<double, double>> new_points = points;

  for (const auto &neighbor : new_neighbors) {
    for (size_t i = 0; i < new_points.size(); ++i) {
      double distance = std::sqrt(std::pow(new_points[i].first - neighbor.first, 2) + std::pow(new_points[i].second - neighbor.second, 2));

      if (distance > maximum_distance_conn) {
        index.push_back(i);
      }
    }
  }
  std::vector<std::pair<double, double>> filtered_points;
  for (size_t i = 0; i < new_points.size(); ++i) {
    if (std::find(index.begin(), index.end(), i) == index.end()) {
      filtered_points.push_back(new_points[i]);
    }
  }
  return filtered_points;
}


std::vector<std::pair<double, double>> RBLController::find_closest_points(const std::pair<double, double>              &robot_pos,
                                                                          const std::vector<std::pair<double, double>> &points,
                                                                          const std::vector<std::pair<double, double>> &neighbors,
                                                                          const std::vector<double>                    &only_robots) {

  std::vector<std::pair<double, double>> closer_points;
  int                                    idx = only_robots.size();
  // Print the combined vector
  for (size_t i = 0; i < points.size(); ++i) {
    bool is_closer = true;
    for (size_t j = 0; j < neighbors.size(); ++j) {  // Start from the neighbor at index idx + 1
      const auto &neigh    = neighbors[j];           // Get the neighbor at the j-th index
      double      cwvd     = 0.5;
      double      alpha_ij = std::atan2(neigh.second - robot_pos.second, neigh.first - robot_pos.first);
      if (j >= neighbors.size() - obstacles_.size()) {
        cwvd = cwvd_obs;
      } else {
        cwvd = cwvd_rob;
      }
      // Check the condition using alpha_ij and neighbor position
      if (std::cos(alpha_ij) * (points[i].first - robot_pos.first) + std::sin(alpha_ij) * (points[i].second - robot_pos.second) >
          cwvd * (std::sqrt(std::pow(robot_pos.first - neigh.first, 2) + std::pow(robot_pos.second - neigh.second, 2)))) {
        is_closer = false;
        break;  // Break if the condition is met
      }
    }
    if (is_closer)
      closer_points.push_back(points[i]);
  }
  return closer_points;
}

std::vector<double> RBLController::compute_scalar_value(const std::vector<double> &x_test, const std::vector<double> &y_test,
                                                        const std::pair<double, double> &destination, double beta) {
  std::vector<double> scalar_values;
  for (size_t i = 0; i < x_test.size(); ++i) {
    double distance     = std::sqrt(std::pow((x_test[i] - destination.first), 2) + std::pow((y_test[i] - destination.second), 2));
    double scalar_value = std::exp(-distance / beta);
    scalar_values.push_back(scalar_value);
  }
  return scalar_values;
}

std::vector<std::pair<double, double>> RBLController::account_encumbrance(const std::vector<std::pair<double, double>> &points,
                                                                          const std::pair<double, double>              &robot_pos,
                                                                          const std::vector<std::pair<double, double>> &neighbors,
                                                                          const std::vector<double> &size_neighbors, double encumbrance) {
  std::vector<size_t> index;
  double              robot_x = robot_pos.first;
  double              robot_y = robot_pos.second;
  for (size_t j = 0; j < neighbors.size(); ++j) {

    double delta_x = robot_x - neighbors[j].first;
    double delta_y = robot_y - neighbors[j].second;

    if (std::abs(delta_y) < 0.001) {
      delta_y = 0.001;
    }
    if (std::abs(delta_x) < 0.001) {
      delta_x = 0.001;
    }

    double m = delta_y / delta_x;
    if (std::abs(m) < 0.001) {
      m = 0.001;
    }
    double xm = 0.5 * (robot_x + neighbors[j].first);
    double ym = 0.5 * (robot_y + neighbors[j].second);
    double dm = std::sqrt(std::pow(xm - robot_x, 2) + std::pow(ym - robot_y, 2));

    if (dm / 2 < (size_neighbors[j] + encumbrance)) {
      std::vector<double> uvec      = {delta_x, delta_y};
      double              norm_uvec = std::sqrt(std::pow(delta_x, 2) + std::pow(delta_y, 2));
      uvec[0] /= norm_uvec;
      uvec[1] /= norm_uvec;

      double solx = xm + (size_neighbors[j] + encumbrance - dm) * uvec[0];
      double soly = ym + (size_neighbors[j] + encumbrance - dm) * uvec[1];

      if (robot_y + (1 / m) * (robot_x - solx) - soly > 0) {
        for (size_t i = 0; i < points.size(); ++i) {
          if (points[i].second + (1 / m) * (points[i].first - solx) - soly < 0) {
            index.push_back(i);
          }
        }
      } else {
        for (size_t i = 0; i < points.size(); ++i) {
          if (points[i].second + (1 / m) * (points[i].first - solx) - soly > 0) {
            index.push_back(i);
          }
        }
      }
    }
  }

  std::vector<std::pair<double, double>> new_points;
  for (size_t i = 0; i < points.size(); ++i) {
    if (std::find(index.begin(), index.end(), i) == index.end()) {
      new_points.push_back(points[i]);
    }
  }
  return new_points;
}
// Function to check if a point is inside a convex polygon
bool RBLController::isInsideConvexPolygon(const std::vector<std::pair<double, double>> &polygon, const std::pair<double, double> &testPoint) {
  int  n      = polygon.size();
  bool inside = false;

  // Ray casting algorithm
  for (int i = 0, j = n - 1; i < n; j = i++) {
    if (((polygon[i].second > testPoint.second) != (polygon[j].second > testPoint.second)) &&
        (testPoint.first <
         (polygon[j].first - polygon[i].first) * (testPoint.second - polygon[i].second) / (polygon[j].second - polygon[i].second) + polygon[i].first)) {
      inside = !inside;
    }
  }

  return inside;
}

std::tuple<std::pair<double, double>, std::pair<double, double>, std::pair<double, double>> RBLController::get_centroid(
    std::pair<double, double> robot_pos, double radius, double step_size, std::vector<std::pair<double, double>> &neighbors,
    const std::vector<double> &size_neighbors, std::vector<std::pair<double, double>> &neighbors_and_obstacles,
    const std::vector<double> &size_neighbors_and_obstacles, double encumbrance, const std::pair<double, double> &destination, double &beta,
    std::vector<std::deque<double>> &x_windows, std::vector<std::deque<double>> &y_windows,
    std::vector<std::pair<double, double>> &neighbors_and_obstacles_noisy) {

  neighbors_and_obstacles_noisy.clear();
  neighbors_and_obstacles_noisy = neighbors_and_obstacles;
  // Get points inside the circle
  std::vector<std::pair<double, double>> circle_points = points_inside_circle(robot_pos, radius, step_size);

  std::vector<std::pair<double, double>> voronoi_circle_intersection;
  std::vector<std::pair<double, double>> voronoi_circle_intersection_connectivity;

  if (!neighbors_and_obstacles_noisy.empty()) {
    // Compute the Voronoi cell
    voronoi_circle_intersection = find_closest_points(robot_pos, circle_points, neighbors_and_obstacles_noisy, size_neighbors);

    // Account encumbrance
    voronoi_circle_intersection =
        account_encumbrance(voronoi_circle_intersection, robot_pos, neighbors_and_obstacles_noisy, size_neighbors_and_obstacles, encumbrance);

    all_uavs = insert_pair_at_index(neighbors, this_uav_idx_, val);

    fixed_neighbors_vec = fixed_neighbors(all_uavs, Adj_matrix, this_uav_idx_);
    if (connectivity_flag==true)
    {
      voronoi_circle_intersection_connectivity = communication_constraint(voronoi_circle_intersection, fixed_neighbors_vec);
    }else{
      voronoi_circle_intersection_connectivity = voronoi_circle_intersection;
    }
    if (voronoi_circle_intersection_connectivity.empty())

    {
      std::cout << "WARNING" << std::endl;
      voronoi_circle_intersection_connectivity.push_back(robot_pos);
    }
  } else {
    voronoi_circle_intersection_connectivity = circle_points;
  }

  std::vector<double> x_in, y_in;
  for (const auto &point : voronoi_circle_intersection_connectivity) {
    x_in.push_back(point.first);
    y_in.push_back(point.second);
  }

  std::vector<double> x_in_no_neigh, y_in_no_neigh;
  for (const auto &point : circle_points) {
    x_in_no_neigh.push_back(point.first);
    y_in_no_neigh.push_back(point.second);
  }

  std::vector<double> x_in_no_conn, y_in_no_conn;
  for (const auto &point : voronoi_circle_intersection) {
    x_in_no_conn.push_back(point.first);
    y_in_no_conn.push_back(point.second);
  }

  // Compute scalar values
  std::vector<double> scalar_values          = compute_scalar_value(x_in, y_in, destination, beta);
  std::vector<double> scalar_values_no_neigh = compute_scalar_value(x_in_no_neigh, y_in_no_neigh, destination, beta);
  std::vector<double> scalar_values_no_conn  = compute_scalar_value(x_in_no_conn, y_in_no_conn, destination, beta);

  // Compute the weighted centroid
  double sum_x_in_times_scalar_values = 0.0;
  double sum_y_in_times_scalar_values = 0.0;
  double sum_scalar_values            = 0.0;

  for (size_t i = 0; i < x_in.size(); ++i) {
    sum_x_in_times_scalar_values += x_in[i] * scalar_values[i];
    sum_y_in_times_scalar_values += y_in[i] * scalar_values[i];
    sum_scalar_values += scalar_values[i];
  }

  std::pair<double, double> centroid = std::make_pair(sum_x_in_times_scalar_values / sum_scalar_values, sum_y_in_times_scalar_values / sum_scalar_values);

  std::vector<Point> hull = convexHull(voronoi_circle_intersection_connectivity);
  double             distance;
  hull_voro.clear();
  for (const Point &p : hull) {
    hull_voro.push_back(std::make_pair(p.first, p.second));

    distance = euclideanDistance(p, centroid);
    // TODO: value of the threshold should change depending on covarince matrices and positions of neighbors

    // if (voronoi_circle_intersection_connectivity.empty()){
    while (distance < threshold) {

      beta                               = beta + dt * 0.1;
      std::vector<double> scalar_values1 = compute_scalar_value(x_in, y_in, destination, beta);
      // Compute the weighted centroid
      double sum_x_in_times_scalar_values1 = 0.0;
      double sum_y_in_times_scalar_values1 = 0.0;
      double sum_scalar_values1            = 0.0;

      for (size_t i = 0; i < x_in.size(); ++i) {
        sum_x_in_times_scalar_values1 += x_in[i] * scalar_values1[i];
        sum_y_in_times_scalar_values1 += y_in[i] * scalar_values1[i];
        sum_scalar_values1 += scalar_values1[i];
      }
      centroid = std::make_pair(sum_x_in_times_scalar_values1 / sum_scalar_values1, sum_y_in_times_scalar_values1 / sum_scalar_values1);
      distance = euclideanDistance(p, centroid);
      // std::cout << "Distance is less than the threshold, dist: " << distance << " beta: " << beta << std::endl;
      // std::cout << "size cell " << voronoi_circle_intersection_connectivity.size() << std::endl;
      // std::cout << "size cell " << voronoi_circle_intersection.size() << std::endl;

      // auto centroids = RBLController::get_centroid(robot_pos, radius, step_size, neighbors, size_neighbors, neighbors_and_obstacles,
      // size_neighbors_and_obstacles, encumbrance, destination, beta, dist_windows, angle_windows);
      // break;
      if (beta > 20.0) {
        break;
      }
    }
  }
  // Compute the centroid without neighbors
  double sum_x_in_no_neigh_times_scalar_values = 0.0;
  double sum_y_in_no_neigh_times_scalar_values = 0.0;
  double sum_scalar_values_no_neigh            = 0.0;

  for (size_t i = 0; i < x_in_no_neigh.size(); ++i) {
    sum_x_in_no_neigh_times_scalar_values += x_in_no_neigh[i] * scalar_values_no_neigh[i];
    sum_y_in_no_neigh_times_scalar_values += y_in_no_neigh[i] * scalar_values_no_neigh[i];
    sum_scalar_values_no_neigh += scalar_values_no_neigh[i];
  }

  std::pair<double, double> centroid_no_neigh =
      std::make_pair(sum_x_in_no_neigh_times_scalar_values / sum_scalar_values_no_neigh, sum_y_in_no_neigh_times_scalar_values / sum_scalar_values_no_neigh);

  // Compute the centroid without conn
  double sum_x_in_no_conn_times_scalar_values = 0.0;
  double sum_y_in_no_conn_times_scalar_values = 0.0;
  double sum_scalar_values_no_conn            = 0.0;

  for (size_t i = 0; i < x_in_no_conn.size(); ++i) {
    sum_x_in_no_conn_times_scalar_values += x_in_no_conn[i] * scalar_values_no_conn[i];
    sum_y_in_no_conn_times_scalar_values += y_in_no_conn[i] * scalar_values_no_conn[i];
    sum_scalar_values_no_conn += scalar_values_no_conn[i];
  }

  std::pair<double, double> centroid_no_conn =
      std::make_pair(sum_x_in_no_conn_times_scalar_values / sum_scalar_values_no_conn, sum_y_in_no_conn_times_scalar_values / sum_scalar_values_no_conn);

  return std::make_tuple(centroid, centroid_no_neigh, centroid_no_conn);
}

void RBLController::apply_rules(double &beta, const std::vector<double> &c1, const std::vector<double> &c2, const std::vector<double> &current_position,
                                double dt, double beta_min, const double &betaD, std::vector<double> &goal, double d1, double &th, double d2, double d3,
                                double d4, std::pair<double, double> &destinations, std::vector<double> &c1_no_rotation) {

  // Extract x, y components from current_position
  double current_j_x = current_position[0];
  double current_j_y = current_position[1];

  // first condition

  double dist_c1_c2 = sqrt(pow((c1[0] - c2[0]), 2) + pow((c1[1] - c2[1]), 2));
  if (dist_c1_c2 > d2 && sqrt(pow((current_j_x - c1[0]), 2) + pow((current_j_y - c1[1]), 2)) < d1) {
    beta = std::max(beta - dt, beta_min);
  } else {
    beta = beta - dt * (beta - betaD);
  }

   std::cout <<  "distp_c1 = " << sqrt(pow((current_j_x - c1[0]), 2) + pow((current_j_y - c1[1]), 2)) << std::endl;

  // second condition
  bool dist_c1_c2_d4 = dist_c1_c2 > d4;
  if (dist_c1_c2_d4 && sqrt(pow((current_j_x - c1[0]), 2) + pow((current_j_y - c1[1]), 2)) < d3) {
    th = std::min(th + dt, M_PI / 2);
    /* std::cout << "RHSrule" << std::endl; */
  } else {
    th = std::max(0.0, th - 2 * dt);
  }

  // third condition
  if (th == M_PI / 2 && sqrt(pow((current_j_x - c1_no_rotation[0]), 2) + pow((current_j_y - c1_no_rotation[1]), 2)) >
                            sqrt(pow((current_j_x - c1[0]), 2) + pow((current_j_y - c1[1]), 2))) {
    th = 0;
    // std::cout << "reset" << std::endl;
  }
  /* std::cout << "theta : " << th << ", beta: " << beta << std::endl; */
  // Compute the angle and new position
  double angle        = atan2(goal[1] - current_j_y, goal[0] - current_j_x);
  double new_angle    = angle - th;
  double distance     = sqrt(pow((goal[0] - current_j_x), 2) + pow((goal[1] - current_j_y), 2));
  destinations.first  = current_j_x + distance * cos(new_angle);
  destinations.second = current_j_y + distance * sin(new_angle);
  // to add rule: change goal?
}
//}

/* getPositionCmd() //{ */
void RBLController::getPositionCmd() {

  if (!is_initialized_) {
    return;
  }

  if (!sh_position_command_.hasMsg()) {
    return;
  }

  mrs_msgs::TrackerCommand    msg = *sh_position_command_.getMsg();
  geometry_msgs::PointStamped new_point;

  new_point.header = msg.header;
  new_point.point  = msg.position;

  auto res = transformer_->transformSingle(new_point, _control_frame_);
  if (res) {
    new_point = res.value();
  } else {
    ROS_ERROR_THROTTLE(3.0, "[RBLController]: Could not transform position command to control frame.");
    return;
  }
  mrs_lib::set_mutexed(mutex_position_command_, new_point.point, position_command_);
  got_position_command_ = true;
}

//}

/* odomCallback() //{ */

void RBLController::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {

  if (!is_initialized_) {
    return;
  }

  geometry_msgs::PointStamped new_point;

  new_point.header  = msg->header;
  new_point.point.x = msg->pose.pose.position.x;
  new_point.point.y = msg->pose.pose.position.y;
  new_point.point.z = msg->pose.pose.position.z;

  /* auto res = transformer_->transformSingle(new_point, _control_frame_); */
  /* if (res) { */
  /*   new_point = res.value(); */
  /* } else { */
  /*   ROS_ERROR_THROTTLE(3.0, "[RBLController]: Could not transform odometry msg to control frame."); */
  /*   return; */
  /* } */

  Eigen::Vector3d transformed_position;
  if (_c_dimensions_ == 3) {
    transformed_position = Eigen::Vector3d(new_point.point.x, new_point.point.y, new_point.point.z);
  } else {
    transformed_position = Eigen::Vector3d(new_point.point.x, new_point.point.y, 0.0);
  }
  mrs_lib::set_mutexed(mutex_uav_odoms_, transformed_position, uav_position_);


  /*   uav_position_[0] = msg->pose.pose.position.x; */
  /*   uav_position_[1] = msg->pose.pose.position.y; */
  /*   uav_position_[2] = msg->pose.pose.position.z; */
}

//}


/* odomCallback() //{ */
void RBLController::odomCallback(const nav_msgs::OdometryConstPtr &msg, int idx) {

  // if (!is_initialized_)
  // {
  //   return;
  // }

  /* nav_msgs::Odometry msg = *sh_ptr.getMsg(); */

  if ((ros::Time::now() - msg->header.stamp).toSec() > _odom_msg_max_latency_)
  {
    ROS_WARN("[RBLController]: The latency of odom message for %s exceeds the threshold (latency = %.2f s).", _uav_names_[idx].c_str(),
             (ros::Time::now() - msg->header.stamp).toSec());
  }

  geometry_msgs::PointStamped new_point;

  new_point.header = msg->header;
  new_point.point.x = msg->pose.pose.position.x;
  new_point.point.y = msg->pose.pose.position.y;
  new_point.point.z = msg->pose.pose.position.z;
  // std::cout <<"x: "<< new_point.point.x  <<"y: "<<new_point.point.y<< std::endl;
  // auto res = transformer_->transformSingle(new_point, _control_frame_);
  // if (res)
  // {
  //   new_point = res.value();
  // }
  // else
  // {
  //   ROS_ERROR_THROTTLE(3.0, "[RBLController]: Could not transform odometry msg to control frame.");
  //   return;
  // }

  // std::cout <<"x: "<< new_point.point.x <<"y:" <<new_point.point.y<< std::endl;
  Eigen::Vector3d transformed_position;
  if (_c_dimensions_ == 3)
  {
    transformed_position = Eigen::Vector3d(new_point.point.x, new_point.point.y, new_point.point.z);
  }
  else
  {
    transformed_position = Eigen::Vector3d(new_point.point.x, new_point.point.y, 0.0);
  }

  mrs_lib::set_mutexed(mutex_uav_odoms_, transformed_position, uav_positions_[idx]);
  mrs_lib::set_mutexed(mutex_uav_odoms_, transformed_position, uav_neighbors_[idx]);

  /* last_odom_msg_time_[idx] = ros::Time::now(); */

}
//}

/* clustersCallback() //{ */
void RBLController::clustersCallback(const visualization_msgs::MarkerArray::ConstPtr &marker_array_msg) {
  // Clear the previous list of obstacles

  std::scoped_lock lock(mutex_obstacles_);
  if (simulation_) {
    obstacles_.clear();
  }

  // obstacles_.shrink_to_fit();
  // Iterate through markers to compute centroids
  for (const auto &marker : marker_array_msg->markers) {
    std::pair<double, double> closest_point;
    double                    min_distance = std::numeric_limits<double>::max();
    if (marker.type == visualization_msgs::Marker::POINTS) {
      for (const auto &point : marker.points) {
        geometry_msgs::PointStamped point_transformed;
        point_transformed.header  = marker.header;
        point_transformed.point.x = point.x;
        point_transformed.point.y = point.y;
        point_transformed.point.z = point.z;

        // Transform the point to the control frame
        auto res = transformer_->transformSingle(point_transformed, _control_frame_);
        if (res) {
          point_transformed = res.value();
        } else {
          ROS_ERROR_THROTTLE(3.0, "[RBLController]: Could not transform obstacle point to control frame.");
          return;
        }

        // Compute distance to UAV position
        double dx       = point_transformed.point.x - uav_position_[0];
        double dy       = point_transformed.point.y - uav_position_[1];
        double distance = std::sqrt(dx * dx + dy * dy);

        // Check if this is the closest point so far
        if (distance < min_distance) {
          min_distance  = distance;
          closest_point = std::make_pair(point_transformed.point.x, point_transformed.point.y);
        }
      }
      // Store the point in obstacles vector
      if (min_distance < 15.0) {
        obstacles_.emplace_back(closest_point.first, closest_point.second);
      }
    }
    /* // Compute the centroid of the points in this marker */
    /* double x_sum      = 0.0; */
    /* double y_sum      = 0.0; */
    /* int    num_points = marker.points.size(); */
    /* double dist       = 1000; */
    /* for (const auto &point : marker.points) { */
    /*   x_sum += point.x; */
    /*   y_sum += point.y; */
    /*   /1* if (sqrt((point.x - robot_x) * (point.x - robot_x) + (point.y - robot_y)(point.y - robot_y)) < dist) *1/ */
    /*   /1*   dist = sqrt((point.x - robot_x)*(point.x - robot_x) + (point.y - robot_y)(point.y - robot_y) *1/ */
    /*   /1*   x_sum = point.x; *1/ */
    /*   /1*   y_sum = point.y; *1/ */
    /* } */

    /* if (num_points > 0) { */
    /*   geometry_msgs::PointStamped new_point; */
    /*   new_point.header  = marker.header; */
    /*   new_point.point.x = x_sum / num_points; */
    /*   new_point.point.y = y_sum / num_points; */
    /*   new_point.point.z = 0; */

    /*   auto res = transformer_->transformSingle(new_point, _control_frame_); */
    /*   if (res) { */
    /*     new_point = res.value(); */
    /*   } else { */
    /*     ROS_ERROR_THROTTLE(3.0, "[RBLController]: Could not transform obstacle centroids to control frame."); */
    /*     return; */
    /*   } */
    /*   // Store centroid in obstacles vector */

    /*   obstacles_.emplace_back(new_point.point.x, new_point.point.y); */
    /*   /1* std::cout << "Number of seen obstacles seen: "<< obstacles_.size()  << std::endl; *1/ */
    /* } */
  }
}

//}

/* clustersCallback1() //{ */
void RBLController::clustersCallback1(const visualization_msgs::MarkerArray::ConstPtr &marker_array_msg) {
  // Clear the previous list of obstacles
  std::scoped_lock lock(mutex_obstacles_);
  if (simulation_) {
    obstacles_.clear();
  }

  // Iterate through markers
  for (const auto &marker : marker_array_msg->markers) {
    std::pair<double, double> closest_point;
    double                    min_distance = std::numeric_limits<double>::max();
    if (marker.type == visualization_msgs::Marker::POINTS) {
      // Process all points in this marker
      for (const auto &point : marker.points) {
        geometry_msgs::PointStamped point_transformed;
        point_transformed.header  = marker.header;
        point_transformed.point.x = point.x;
        point_transformed.point.y = point.y;
        point_transformed.point.z = point.z;

        // Transform the point to the control frame
        auto res = transformer_->transformSingle(point_transformed, _control_frame_);
        if (res) {
          point_transformed = res.value();
        } else {
          ROS_ERROR_THROTTLE(3.0, "[RBLController]: Could not transform obstacle point to control frame.");
          return;
        }

        // Compute distance to UAV position
        double dx       = point_transformed.point.x - uav_position_[0];
        double dy       = point_transformed.point.y - uav_position_[1];
        double distance = std::sqrt(dx * dx + dy * dy);

        // Check if this is the closest point so far
        if (distance < min_distance) {
          min_distance  = distance;
          closest_point = std::make_pair(point_transformed.point.x, point_transformed.point.y);
        }
      }
      // Store the point in obstacles vector
      if (min_distance < 15.0) {
        obstacles_.emplace_back(closest_point.first, closest_point.second);
      }
    }
  }
}
/* //} */

/* clustersCallback1() //{ */
/* void RBLController::clustersCallback1(const visualization_msgs::MarkerArray::ConstPtr &marker_array_msg) { */
/*   // Clear the previous list of obstacles */

/*   std::scoped_lock lock(mutex_obstacles_); */
/*   obstacles_.clear(); */
/*   // obstacles_.shrink_to_fit(); */
/*   // Iterate through markers to compute centroids */
/*   for (const auto &marker : marker_array_msg->markers) { */
/*     if (marker.type == visualization_msgs::Marker::POINTS) { */
/*       // Compute the centroid of the points in this marker */
/*       double x_sum      = 0.0; */
/*       double y_sum      = 0.0; */
/*       int    num_points = marker.points.size(); */

/*       for (const auto &point : marker.points) { */
/*         x_sum += point.x; */
/*         y_sum += point.y; */
/*       } */

/*       if (num_points > 0) { */
/*         geometry_msgs::PointStamped new_point; */
/*         new_point.header  = marker.header; */
/*         new_point.point.x = x_sum / num_points; */
/*         new_point.point.y = y_sum / num_points; */
/*         new_point.point.z = 0; */

/*         auto res = transformer_->transformSingle(new_point, _control_frame_); */
/*         if (res) { */
/*           new_point = res.value(); */
/*         } else { */
/*           ROS_ERROR_THROTTLE(3.0, "[RBLController]: Could not transform obstacle centroids to control frame."); */
/*           return; */
/*         } */
/*         // Store centroid in obstacles vector */

/*         obstacles_.emplace_back(new_point.point.x, new_point.point.y); */
/*         /1* obstacles_.insert(obstacles_.end(),obstacles1_.begin(),obstacles1_.end()); *1/ */
/*         /1* std::cout << "Number of seen obstacles seen: "<< obstacles_.size()  << std::endl; *1/ */
/*       } */
/*     } */
/*   } */
/* } */
//}

/* /1* waypointsCallback() //{ *1/ */
/* void RBLController::waypointsCallback1(const visualization_msgs::MarkerArray::ConstPtr &marker_array_msg) { */
/*   // Clear the previous list of obstacles */
/*   std::scoped_lock lock(mutex_waypoints_); */

/*   std::cout << "ciao0" << std::endl; */
/*   // obstacles_.shrink_to_fit(); */
/*   // Iterate through markers to compute centroids */
/*   for (const auto &marker : marker_array_msg->markers) { */
/*     /1* if (marker.type == visualization_msgs::Marker::POINTS) { *1/ */
/*     std::cout << "ciao1" << std::endl; */
/*     for (const auto &point : marker.points) { */
/*       std::cout << "ciao2" << std::endl; */
/*       geometry_msgs::PointStamped point_transformed; */
/*       point_transformed.header  = marker.header; */
/*       point_transformed.point.x = point.x; */
/*       point_transformed.point.y = point.y; */
/*       point_transformed.point.z = point.z; */

/*       // Transform the point to the control frame */
/*       auto res = transformer_->transformSingle(point_transformed, _control_frame_); */
/*       if (res) { */
/*         point_transformed = res.value(); */
/*       } else { */
/*         ROS_ERROR_THROTTLE(3.0, "[RBLController]: Could not transform obstacle point to control frame."); */
/*         return; */
/*       } */
/*       waypoints_.emplace_back(point_transformed.point.x, point_transformed.point.y); */
/*       std::cout << "sizeeee: " << waypoints_.size() << std::endl; */
/*       /1* } *1/ */
/*     } */
/*   } */
/* } */

/* //} */

/* waypointsCallback() //{ */
void RBLController::waypointsCallback(const visualization_msgs::MarkerArray::ConstPtr &marker_array_msg) {
  // Clear the waypoints vector before populating it with new data
  waypoints_.clear();

  // Loop through the markers in the MarkerArray
  for (const auto &marker : marker_array_msg->markers) {
    // Ensure that the marker has a valid position (check if position is available)
    if (marker.pose.position.x && marker.pose.position.y) {
      // Extract the x, y coordinates from the marker pose and store them in waypoints_
      waypoints_.emplace_back(marker.pose.position.x, marker.pose.position.y);
    }
  }

  // Debug print to see the waypoints received (optional)
  /* ROS_INFO_STREAM("Received " << waypoints_.size() << " waypoints:"); */
  /* for (const auto &waypoint : waypoints_) { */
  /*   ROS_INFO_STREAM("Waypoint: (" << waypoint.first << ", " << waypoint.second << ")"); */
}

//}

/*RBLController::callbackNeighborsUsingUVDAR() //{ */
void RBLController::callbackNeighborsUsingUVDAR(const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr &array_poses) {

  // ROS_INFO("Received pose from uvdar");
  /* uav_neighbors_.assign(n_drones_ + 1, Eigen::Vector3d(1000, 0, 0)); */
  largest_eigenvalue_.assign(n_drones_ + 1, 0);
  Eigen::Matrix2d CovarianceMatrix;
  for (int i = 0; i < array_poses->poses.size(); i++) {
    /* create new msg */
    geometry_msgs::PointStamped new_point;

    Eigen::Vector3d transformed_position;
    new_point.header  = array_poses->header;
    new_point.point.x = array_poses->poses[i].pose.position.x;
    new_point.point.y = array_poses->poses[i].pose.position.y;
    new_point.point.z = array_poses->poses[i].pose.position.z;
    int uav_id        = array_poses->poses[i].id;
    CovarianceMatrix << array_poses->poses[i].covariance[0], array_poses->poses[i].covariance[1], array_poses->poses[i].covariance[6],
        array_poses->poses[i].covariance[7];

    auto res = transformer_->transformSingle(new_point, _control_frame_);
    if (res) {
      new_point = res.value();
    } else {
      ROS_ERROR_THROTTLE(3.0, "[RBLController]: UVDAR: Could not transform positions to control frame.");
      continue;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(CovarianceMatrix);
    Eigen::Vector2d                                eigenvalues        = solver.eigenvalues();
    double                                         largest_eigenvalue = eigenvalues.maxCoeff();
    double valeig  = std::abs(std::abs(std::sqrt(pow((new_point.point.x - position_command_.x), 2) + pow((new_point.point.y - position_command_.y), 2))) / 2);
    double valeig1 = 2.6 * std::sqrt(largest_eigenvalue) + encumbrance;

    /* std::cout << "Largest eigenvalue: " << uav_id << " eig: " << largest_eigenvalue << std::endl; */

    if (_c_dimensions_ == 3) {
      transformed_position = Eigen::Vector3d(new_point.point.x, new_point.point.y, new_point.point.z);
    } else {
      transformed_position = Eigen::Vector3d(new_point.point.x, new_point.point.y, 0.0);
    }
    has_this_pose_ = true;
    if (largest_eigenvalue < 15.0) {
      mrs_lib::set_mutexed(mutex_uav_uvdar_, transformed_position, uav_neighbors_[_uav_uvdar_ids_[uav_id]]);
      mrs_lib::set_mutexed(mutex_uav_uvdar_, largest_eigenvalue, largest_eigenvalue_[_uav_uvdar_ids_[uav_id]]);
    }
  }
}
//}

/* callbackTimerPubNeighbors() //{ */

void RBLController::callbackTimerPubNeighbors([[maybe_unused]] const ros::TimerEvent &event) {
  if (!is_initialized_ || !has_this_pose_) {
    return;
  }
}


//}

/* callbackTimerSetReference() Callback where we set the p_ref //{ */
void RBLController::callbackTimerSetReference([[maybe_unused]] const ros::TimerEvent &te) {
  if (!is_initialized_) {
    return;
  }

  auto start_time = std::chrono::high_resolution_clock::now();
  if (!got_position_command_) {
    ROS_WARN_THROTTLE(3.0, "[RBLController]: Waiting for valid robots' positions.");
    getPositionCmd();
    /* return; */
  }

  /* if (!is_at_initial_position_) { */

  /*   double dist_to_start = getDistToInitialPosition(); */

  /*   if (dist_to_start > _dist_to_start_limit_) { */

  /*     ROS_WARN_THROTTLE(3.0, "[RBLController]: Waiting for UAV to arrive at initial position. Current distance: %.2f m.", dist_to_start); */
  /*     return; */
  /*   } else { */

  /*     ROS_INFO("[RBLController]: UAV arrived to its initial position."); */
  /*     is_at_initial_position_ = true; */
  /*   } */
  /* } */

  if (!control_allowed_) {
    ROS_WARN_THROTTLE(3.0, "[RBLController]: Waiting for activation.");
    return;
  }

  getPositionCmd();

  /* TODO: calculate desired drone velocity or positional reference here */

  mrs_msgs::Reference p_ref;

  {
    std::scoped_lock                       lock(mutex_uav_odoms_, mutex_position_command_, mutex_uav_uvdar_);
    auto                                   start = std::chrono::steady_clock::now();
    std::vector<std::pair<double, double>> neighbors;
    std::vector<std::pair<double, double>> neighbors_and_obstacles;
    robot_pos = {
        uav_position_[0], uav_position_[1],
        /* position_command_.x, */
        /* position_command_.y, */
    };
    double distance2neigh;
    for (int j = 0; j < n_drones_; ++j) {
      neighbors.push_back({uav_neighbors_[j][0], uav_neighbors_[j][1]});
      neighbors_and_obstacles.push_back({uav_neighbors_[j][0], uav_neighbors_[j][1]});
      /* std::cout << uav_neighbors_[j][0] << ", " << uav_neighbors_[j][1] << std::endl; */
      /* distance2neigh = std::sqrt(std::pow((uav_neighbors_[j][0] - robot_pos.x), 2) + std::pow((uav_neighbors_[j][1] - robot_pos.y), 2)); */
      /* if (largest_eigenvalue_[j] / 2.0 + encumbrance > distance2neigh / 2.0 && distance2neigh < 5.0) { */
      /*   // std::cout<<"theoretical du for uvdar" << (largest_eigenvalue_[j]/2.0 + encumbrance - distance2neigh/2.0)+ encumbrance/2 <<std::endl; */
      /* } */
      /* if (Adj_matrix[j] == 1) { */
      /*   // std::cout<<"eigenvalues_neigh" << largest_eigenvalue_[j]/2 << std::endl; */
      /* } */
    }

    {
      std::scoped_lock lock(mutex_obstacles_);
      for (int j = 0; j < obstacles_.size(); ++j) {
        if (pow(uav_position_[0] - obstacles_[j].first, 2) + pow(uav_position_[1] - obstacles_[j].second, 2) < max_obstacle_integration_dist_sqr_) {
          neighbors_and_obstacles.push_back({obstacles_[j].first, obstacles_[j].second});
        }
      }

      size_neighbors_and_obstacles.clear();
      size_obstacles.assign(obstacles_.size(), size_obstacles1);
    }
    size_neighbors_and_obstacles = size_neighbors;  // Copy vec1 to vec3
    size_neighbors_and_obstacles.insert(size_neighbors_and_obstacles.end(), size_obstacles.begin(), size_obstacles.end());

    x_windows.resize(neighbors_and_obstacles.size());
    y_windows.resize(neighbors_and_obstacles.size());

    // Resize each deque in dist_windows to have a size of 100
    for (auto &x_window : x_windows) {
      x_window.resize(window_length, 0.0);
    }
    for (auto &y_window : y_windows) {
      y_window.resize(window_length, 0.0);
    }

    // TODO: manage end of the path and put the active as the closest to the robot + 5 ...something like that
    if (replanner_flag == true) {
      double min_distance1    = std::numeric_limits<double>::infinity();
      int    closest_wp_index = -1;
      for (int i = 0; i < waypoints_.size(); ++i) {
        double distance = calculateDistance(robot_pos, waypoints_[i]);
        if (distance < min_distance1) {
          min_distance1 = distance;
          closest_wp_index = i;
        }
      }

      // Choose the point n positions ahead of the closest waypoint
      int n             = 8;  // Example: Choose the point 2 positions ahead
      int next_wp_index = closest_wp_index + n;

      // Ensure next_wp_index stays within bounds
      if (next_wp_index >= waypoints_.size()) {
        next_wp_index = waypoints_.size() - 1;  // Take the last waypoint if out of bounds
      }

      // Set the active waypoint
      active_wp = waypoints_[next_wp_index];
      /* active_wp = waypoints_[10]; //to fix 10 */
    } else {
      active_wp = destination;
    }
    /* getClosestWaypoint(robot_pos, 4.0); */
    /* std::cout << "activewp: " << active_wp.first << ", " << active_wp.second <<std::endl; */
    // Call get_centroid function
    /* auto centroids = RBLController::get_centroid(robot_pos, radius, step_size, neighbors, size_neighbors, neighbors_and_obstacles,
       size_neighbors_and_obstacles, */
    /*                                              encumbrance, destination, beta, x_windows, y_windows, neighbors_and_obstacles_noisy); */

    auto centroids = RBLController::get_centroid(robot_pos, radius, step_size, neighbors, size_neighbors, neighbors_and_obstacles, size_neighbors_and_obstacles,
                                                 encumbrance, active_wp, beta, x_windows, y_windows, neighbors_and_obstacles_noisy);
    auto centroids_no_rotation =
        RBLController::get_centroid(robot_pos, radius, step_size, neighbors, size_neighbors, neighbors_and_obstacles, size_neighbors_and_obstacles, encumbrance,
                                    {goal[0], goal[1]}, beta, x_windows, y_windows, neighbors_and_obstacles_noisy);

    std::vector<double> c1         = {std::get<0>(centroids).first, std::get<0>(centroids).second};
    std::vector<double> c2         = {std::get<1>(centroids).first, std::get<1>(centroids).second};
    std::vector<double> c1_no_conn = {std::get<2>(centroids).first, std::get<2>(centroids).second};

    std::vector<double> c1_no_rotation = {std::get<2>(centroids_no_rotation).first, std::get<2>(centroids_no_rotation).second};

    std::vector<double> current_position(2);  // Vector with two elements
    current_position[0] = robot_pos.first;    // Assigning first element
    current_position[1] = robot_pos.second;
    // std::vector<double> destinations = {destination.first, destination.second};
    /* replanning_basic(robot_pos, centroid, neighbors, goal); */
    /* RBLController::replanning_basic(robot_pos, c1, neighbors_and_obstacles_noisy, goal, goal_original); */
    // Call apply_rules function

    RBLController::apply_rules(beta, c1_no_conn, c2, current_position, dt, beta_min, betaD, goal, d1, th, d2, d3, d4, destination, c1_no_rotation);

    p_ref.position.x = c1[0];  // next_values[0];
    p_ref.position.y = c1[1];
    p_ref.position.z = refZ_;

    /* p_ref.heading = std::atan2(destination.second - robot_pos.second, destination.first - robot_pos.first); //- 3.1415 / 4; */
    p_ref.heading = 0.0;

    // Output the duration
    /* std::cout << "Time taken: " << duration.count() << " milliseconds" << std::endl; */
  }
  /* ROS_INFO_THROTTLE(3.0, "[RBLController]: Setting positional reference [%.2f, %.2f, %.2f] in frame %s, heading = %.2f.", p_ref.position.x, p_ref.position.y,
   */
  /*                   p_ref.position.z, _control_frame_.c_str(), p_ref.heading); */

  // set drone velocity
  // TMP

  mrs_msgs::ReferenceStampedSrv srv;
  srv.request.reference       = p_ref;
  srv.request.header.frame_id = _control_frame_;
  srv.request.header.stamp    = ros::Time::now();

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  /* ROS_INFO("Node took %ld milliseconds", duration.count()); */

  if (sc_set_position_.call(srv)) {
    /* ROS_INFO_THROTTLE(3.0, "Success: %d", srv.response.success); */
    /* ROS_INFO_THROTTLE(3.0, "Message: %s", srv.response.message.c_str()); */
  } else {
    ROS_ERROR_THROTTLE(3.0, "Failed to call service ref_pos_out");
  }

  if (!flag_stop && robot_pos.second > 30.0) {
    flag_stop                    = true;
    ros::Time     end_time_1     = ros::Time::now();
    ros::Duration elapsed_time_1 = end_time_1 - start_time_1;
    ROS_INFO("Elapsed time: %.2f seconds", elapsed_time_1.toSec());
  }
}
//}

/* callbackTimerDiagnostics() //{ */
void RBLController::callbackTimerDiagnostics([[maybe_unused]] const ros::TimerEvent &te) {

  if (!is_initialized_) {
    return;
  }

  bool timeout_exceeded = false;


  try {
    publishDestination();
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_destination_.getTopic().c_str());
  }

  try {
    publishPosition();
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_position_.getTopic().c_str());
  }
  try {
    publishLine();
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_line_.getTopic().c_str());
  }

  try {
    publishCentroid();
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_centroid_.getTopic().c_str());
  }
  try {
    publishObstacles();
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_obstacles_.getTopic().c_str());
  }

  try {
    publishNeighbors();
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_neighbors_.getTopic().c_str());
  }

  try {
    publishHull();
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_hull_.getTopic().c_str());
  }
}
//}

/*ServicesCallbacks //{ */

/* activationServiceCallback() //{ */
bool RBLController::activationServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  // service for activation of planning
  ROS_INFO("[RBLController]: Activation service called.");
  res.success = true;

  if (control_allowed_) {
    res.message = "Control was already allowed.";
    ROS_WARN("[RBLController]: %s", res.message.c_str());
  } else {
    control_allowed_ = true;
    res.message      = "Control allowed.";
    ROS_INFO("[RBLController]: %s", res.message.c_str());
  }

  start_time_1 = ros::Time::now();
  ROS_INFO("Start time activation: %.2f seconds", start_time_1.toSec());
  return true;
}
//}


bool RBLController::activationParamsServiceCallback(rbl_controller::ActivateParams::Request &req,
                               rbl_controller::ActivateParams::Response &res)
{

    // Your logic
    // ROS_INFO("Received: %f %f %f %f %f", a, b, c, d, e);

    res.success = true;
    res.message = "Parameters received and processed.";
    
    destination.first  = req.x;
    destination.second = req.y;

    goal[0]= req.x;
    goal[1]= req.y;
    refZ_ = req.z;
    betaD = req.betaD;
    beta_min = req.beta_min;
    radius = req.radius;
    encumbrance = req.encumbrance;
    size_neighbors1 = req.encumbrance;
    size_neighbors.assign(_uav_names_.size() - 1, size_neighbors1);
    connectivity_flag = req.connectivity;
    cwvd_rob = req.cwvd;
    cwvd_obs = req.cwvd;
    

    control_allowed_ = true;
   // if (control_allowed_) {
    //   res.message = "Control was already allowed.";
    //   ROS_WARN("[RBLController]: %s", res.message.c_str());
    // } else {
    //   res.message      = "Control allowed.";
    //   ROS_INFO("[RBLController]: %s", res.message.c_str());
    // }

    return true;
}
/* deactivationServiceCallback() //{ */
bool RBLController::deactivationServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  // service for deactivation of planning
  ROS_INFO("[RBLController]: Deactivation service called.");
  res.success = true;

  if (!control_allowed_) {
    res.message = "Control was already disables.";
    ROS_WARN("[RBLController]: %s", res.message.c_str());
  } else {
    control_allowed_ = false;
    res.message      = "Control disabled.";
    ROS_INFO("[RBLController]: %s", res.message.c_str());
  }

  return true;
}

//}

/* flyToStartServiceCallback() //{ */
bool RBLController::flyToStartServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  // service for activation of planning
  ROS_INFO("[RBLController]: Fly to start service called.");
  res.success = true;

  mrs_msgs::Vec4 srv;
  srv.request.goal = {_required_initial_position_[0], _required_initial_position_[1], _required_initial_position_[2], 0.0};
  ;
  if (sc_goto_position_.call(srv)) {
    if (srv.response.success) {
      res.message          = "Fly to start called.";
      fly_to_start_called_ = true;
    } else {
      res.success = false;
      res.message = "GoTo service returned success false.";
    }
  } else {
    res.success = false;
    res.message = "Call to GoTo service failed.";
  }

  ROS_INFO("[RBLController]: %s", res.message.c_str());


  return true;
}
//}
//}

// | -------------------- nodelet macro ----------------------- |
PLUGINLIB_EXPORT_CLASS(formation_control::RBLController, nodelet::Nodelet);
}  // namespace formation_control
