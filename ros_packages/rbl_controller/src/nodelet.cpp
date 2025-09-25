#include <geometry_msgs/Point.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_msgs/Reference.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/Vec4.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nodelet/nodelet.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <string>
#include "rbl_controller.h"

class WrapperRosRBL : public nodelet::Nodelet// //{
{
public:
  virtual void onInit();
  bool         is_initialized_ = false;
  bool         is_activated_   = false;

  bool        _group_odoms_enabled_ = false;
  std::string _agent_name_;
  std::string _frame_;

  std::mutex                     mtx_rbl_;
  std::shared_ptr<RBLController> rbl_controller_;
  RBLParams                      rbl_params_;

  ros::ServiceServer srv_activate_control_;
  bool               cbSrvActivateControl([[maybe_unused]] std_srvs::Trigger::Request& req,
                                          std_srvs::Trigger::Response&                 res);
  ros::ServiceServer srv_deactivate_control_;
  bool               cbSrvDeactivateControl([[maybe_unused]] std_srvs::Trigger::Request& req,
                                            std_srvs::Trigger::Response&                 res);
  ros::ServiceServer srv_goto_position_;
  bool               cbSrvGotoPosition(mrs_msgs::Vec4::Request&  req,
                                       mrs_msgs::Vec4::Response& res);

  ros::ServiceClient sc_set_ref_;

  ros::Timer tm_set_ref_;
  void       cbTmSetRef([[maybe_unused]] const ros::TimerEvent& te);
  ros::Timer tm_diagnostics_;
  void       cbTmDiagnostics([[maybe_unused]] const ros::TimerEvent& te);

  ros::Publisher                            pub_viz_cell_A_;
  ros::Publisher                            pub_viz_cell_A_sensed_;
  std::shared_ptr<sensor_msgs::PointCloud2> getVizCellA(const std::vector<Eigen::Vector3d>& points,
                                                        const std::string&                  frame);
  ros::Publisher                            pub_viz_inflated_map_;
  std::shared_ptr<sensor_msgs::PointCloud2> getVizInflatedMap(const std::vector<Eigen::Vector3d>& points,
                                                              const std::string&                  frame);
  ros::Publisher                            pub_viz_path_;
  nav_msgs::Path                            getVizPath(const std::vector<Eigen::Vector3d>& path,
                                                       const std::string&                  frame);
  ros::Publisher                            pub_viz_position_;
  visualization_msgs::Marker                getVizPosition(const Eigen::Vector3d& point,
                                                           const double           scale,
                                                           const std::string&     frame);
  ros::Publisher                            pub_viz_centroid_;
  visualization_msgs::Marker                getVizCentroid(const Eigen::Vector3d& point,
                                                           const std::string&     frame);
  ros::Publisher                            pub_viz_target_;
  visualization_msgs::Marker                getVizModGroupGoal(const Eigen::Vector3d& point,
                                                               const double           scale,
                                                               const std::string&     frame);
  ros::Publisher                            pub_viz_waypoint_;
  visualization_msgs::Marker                getVizWaypoint(const Eigen::Vector3d& point,
                                                           const double           scale,
                                                           const std::string&     frame);


  mrs_lib::SubscribeHandler<nav_msgs::Odometry>       sh_odom_;
  mrs_lib::SubscribeHandler<sensor_msgs::Range>       sh_alt_;
  mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2> sh_pcl_;

  std::vector<mrs_lib::SubscribeHandler<nav_msgs::Odometry>> sh_group_odoms_;

  std::shared_ptr<mrs_lib::Transformer> transformer_;

  Eigen::Vector3d      pointToEigen(const geometry_msgs::Point& point);
  Eigen::Vector3d      vectorToEigen(const geometry_msgs::Vector3& vec);
  geometry_msgs::Point pointFromEigen(const Eigen::Vector3d& vec);
  geometry_msgs::Point createPoint(double x,
                                   double y,
                                   double z);
};// //}

void WrapperRosRBL::onInit()// //{
{
  ros::NodeHandle& nh = getPrivateNodeHandle();
  NODELET_DEBUG("Initializing nodelet...");
  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh, "WrapperRosRBL");

  param_loader.loadParam("uav_name", _agent_name_);
  param_loader.loadParam("control_frame", _frame_);
  param_loader.loadParam("group_odoms/enable", _group_odoms_enabled_);

  std::string odom_topic_name     = param_loader.loadParam2("odometry_topic", "");
  double      rate_tm_set_ref     = param_loader.loadParam2("rate/timer_set_ref", 0.0);
  double      rate_tm_diagnostics = param_loader.loadParam2("rate/timer_diagnostics", 0.0);

  param_loader.loadParam("rbl_controller/only_2d", rbl_params_.only_2d);
  param_loader.loadParam("rbl_controller/z_min", rbl_params_.z_min);
  param_loader.loadParam("rbl_controller/z_max", rbl_params_.z_max);
  param_loader.loadParam("rbl_controller/z_ref", rbl_params_.z_ref);
  param_loader.loadParam("rbl_controller/d1", rbl_params_.d1);
  param_loader.loadParam("rbl_controller/d2", rbl_params_.d2);
  param_loader.loadParam("rbl_controller/d3", rbl_params_.d3);
  param_loader.loadParam("rbl_controller/d4", rbl_params_.d4);
  param_loader.loadParam("rbl_controller/d5", rbl_params_.d5);
  param_loader.loadParam("rbl_controller/d6", rbl_params_.d6);
  param_loader.loadParam("rbl_controller/d7", rbl_params_.d7);
  param_loader.loadParam("rbl_controller/radius", rbl_params_.radius);
  param_loader.loadParam("rbl_controller/encumbrance", rbl_params_.encumbrance);
  param_loader.loadParam("rbl_controller/step_size", rbl_params_.step_size);
  param_loader.loadParam("rbl_controller/betaD", rbl_params_.betaD);
  param_loader.loadParam("rbl_controller/beta_min", rbl_params_.beta_min);
  param_loader.loadParam("rbl_controller/use_z_rule", rbl_params_.use_z_rule);
  param_loader.loadParam("rbl_controller/dt", rbl_params_.dt);
  param_loader.loadParam("rbl_controller/cwvd_rob", rbl_params_.cwvd_rob);
  param_loader.loadParam("rbl_controller/cwvd_obs", rbl_params_.cwvd_obs);
  param_loader.loadParam("rbl_controller/use_garmin_alt", rbl_params_.use_garmin_alt);
  param_loader.loadParam("rbl_controller/replanner", rbl_params_.replanner);
  param_loader.loadParam("rbl_controller/limited_fov", rbl_params_.limited_fov);
  param_loader.loadParam("rbl_controller/ciri", rbl_params_.ciri);
  param_loader.loadParam("rbl_controller/boundary_threshold", rbl_params_.boundary_threshold);
  param_loader.loadParam("rbl_controller/voxel_size", rbl_params_.voxel_size);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[WrapperRosRBL]: Could not load all parameters!");
    ros::shutdown();
  }

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = "WrapperRosRBL";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  if (_group_odoms_enabled_) {

    while (sh_group_odoms_.empty()) {
      ros::master::V_TopicInfo all_topics;
      ros::master::getTopics(all_topics);

      for (const auto& topic : all_topics) {
        if (topic.name.find(_agent_name_) == std::string::npos &&
            topic.name.find(odom_topic_name) != std::string::npos) {
          ROS_INFO_STREAM("[WrapperRosRBL]: Subscribing to topic: " << topic.name);
          sh_group_odoms_.push_back(mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, topic.name.c_str()));
        }
      }

      if (sh_group_odoms_.empty()) {
        ROS_WARN_ONCE("[WrapperRosRBL]: No topics matched: %s", odom_topic_name.c_str());
      }
      else {
        ROS_INFO("[WrapperRosRBL]: Topics matched: %s", odom_topic_name.c_str());
      }
    }
  }

  sh_odom_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "odom_in");
  sh_alt_  = mrs_lib::SubscribeHandler<sensor_msgs::Range>(shopts, "alt_in");
  sh_pcl_  = mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>(shopts, "pcl_in");

  tm_set_ref_     = nh.createTimer(ros::Rate(rate_tm_set_ref), &WrapperRosRBL::cbTmSetRef, this);
  tm_diagnostics_ = nh.createTimer(ros::Rate(rate_tm_diagnostics), &WrapperRosRBL::cbTmDiagnostics, this);

  srv_activate_control_ = nh.advertiseService("control_activation_in", &WrapperRosRBL::cbSrvActivateControl, this);
  srv_goto_position_    = nh.advertiseService("goto_in", &WrapperRosRBL::cbSrvGotoPosition, this);
  srv_deactivate_control_ =
      nh.advertiseService("control_deactivation_in", &WrapperRosRBL::cbSrvDeactivateControl, this);

  sc_set_ref_ = nh.serviceClient<mrs_msgs::ReferenceStampedSrv>("ref_out");

  pub_viz_position_      = nh.advertise<visualization_msgs::Marker>("viz/position", 1, true);
  pub_viz_centroid_      = nh.advertise<visualization_msgs::Marker>("viz/centroid", 1, true);
  pub_viz_cell_A_        = nh.advertise<sensor_msgs::PointCloud2>("viz/cell_a", 1, true);
  pub_viz_cell_A_sensed_ = nh.advertise<sensor_msgs::PointCloud2>("viz/actively_sensed_A", 1, true);
  pub_viz_inflated_map_  = nh.advertise<sensor_msgs::PointCloud2>("viz/inflated_map", 1, true);
  pub_viz_path_          = nh.advertise<nav_msgs::Path>("viz/path", 1, true);
  pub_viz_target_        = nh.advertise<visualization_msgs::Marker>("viz/target", 1, true);
  pub_viz_waypoint_      = nh.advertise<visualization_msgs::Marker>("viz/replanner_waypoint", 1, true);

  transformer_ = std::make_shared<mrs_lib::Transformer>(nh, "WrapperRosRBL");
  transformer_->retryLookupNewest(true);

  {
    std::scoped_lock lck(mtx_rbl_);
    rbl_controller_ = std::make_shared<RBLController>(rbl_params_);
    ROS_INFO("[WrapperRosRBL]: Initialized RBLController with params");
  }

  is_initialized_ = true;
  ROS_INFO("[WrapperRosRBL]: Initialization completed");
}// //}

void WrapperRosRBL::cbTmSetRef([[maybe_unused]] const ros::TimerEvent& te)// //{
{
  if (!is_initialized_) {
    return;
  }

  if (!is_activated_) {
    ROS_WARN_ONCE("[WrapperRosRBL]: Waiting for activation");
    return;
  }

  mrs_msgs::ReferenceStampedSrv msg_ref;
  msg_ref.request.header.frame_id = _frame_;
  msg_ref.request.header.stamp    = ros::Time::now();
  {
    std::scoped_lock lck(mtx_rbl_);
    if (sh_odom_.newMsg()) {
      auto                        odom = sh_odom_.getMsg();
      geometry_msgs::PointStamped tmp_pt;
      tmp_pt.header = odom->header;
      tmp_pt.point  = odom->pose.pose.position;
      auto res = transformer_->transformSingle(tmp_pt, _frame_);

      if (!res) {
        ROS_ERROR_THROTTLE(3.0, "[WrapperRosRBL]: Could not transform odometry msg to control frame.");
        return;
      }
      rbl_controller_->setCurrentPosition(pointToEigen(res.value().point));

      geometry_msgs::Vector3Stamped tmp_vel;
      tmp_vel.header = odom->header;
      tmp_vel.vector = odom->twist.twist.linear;
      auto vel_res = transformer_->transformSingle(tmp_vel, _frame_);

      if (!vel_res) {
        ROS_ERROR_THROTTLE(3.0, "[WrapperRosRBL]: Could not transform velocity to control frame.");
        return;
      }
      rbl_controller_->setCurrentVelocity(vectorToEigen(vel_res->vector));
      Eigen::Vector3d euler;

      double q_x = odom->pose.pose.orientation.x;
      double q_y = odom->pose.pose.orientation.y;
      double q_z = odom->pose.pose.orientation.z;
      double q_w = odom->pose.pose.orientation.w;

      // Roll (X-axis rotation)
      double sinr_cosp = 2.0 * (q_w * q_x + q_y * q_z);
      double cosr_cosp = 1.0 - 2.0 * (q_x * q_x + q_y * q_y);
      euler.x() = std::atan2(sinr_cosp, cosr_cosp);

      // Pitch (Y-axis rotation)
      double sinp = 2.0 * (q_w * q_y - q_z * q_x);
      if (std::abs(sinp) >= 1){
        euler.y() = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
      } else {
        euler.y() = std::asin(sinp);
      }

      // Yaw (Z-axis rotation)
      double siny_cosp = 2.0 * (q_w * q_z + q_x * q_y);
      double cosy_cosp = 1.0 - 2.0 * (q_y * q_y + q_z * q_z);
      euler.z() = std::atan2(siny_cosp, cosy_cosp);

      rbl_controller_->setRollPitchYaw(euler);
    }

    if (sh_alt_.newMsg()) {
      auto alt = sh_alt_.getMsg();
      rbl_controller_->setAltitude(alt->range);
    }

    if (!sh_group_odoms_.empty()) {

      std::vector<Eigen::Vector3d> tmp_odoms;
      std::vector<Eigen::Vector3d> tmp_vels;
      for (auto& tmp_sh : sh_group_odoms_) {

        if (tmp_sh.newMsg()) {
          auto                        odom = tmp_sh.getMsg();
          geometry_msgs::PointStamped tmp_pt;
          tmp_pt.header = odom->header;
          tmp_pt.point  = odom->pose.pose.position;

          auto res = transformer_->transformSingle(tmp_pt, _frame_);

          if (!res) {
            ROS_ERROR_THROTTLE(3.0, "[WrapperRosRBL]: Could not transform odometry msg to control frame.");
            return;
          }
          tmp_odoms.emplace_back(pointToEigen(res.value().point));

          geometry_msgs::Vector3Stamped tmp_vel;
          tmp_vel.header = odom->header;
          tmp_vel.vector = odom->twist.twist.linear;
        
          auto vel_res = transformer_->transformSingle(tmp_vel, _frame_);
          if (!vel_res) {
            ROS_ERROR_THROTTLE(3.0, "[WrapperRosRBL]: Could not transform velocity msg to control frame.");
            return;
          }
          tmp_vels.emplace_back(vectorToEigen(vel_res->vector));  // store velocity
        }
      }
      rbl_controller_->setGroupPositions(tmp_odoms);
    }

    if (sh_pcl_.newMsg()) {
      auto msg = sh_pcl_.getMsg();
      if (msg->header.frame_id != _frame_) {
        ROS_ERROR_STREAM("[WrapperRosRBL]: PCL msg is not in frame: " << _frame_.c_str());
        return;
      }
      rbl_controller_->setPCL(msg);
    }

    auto ret = rbl_controller_->getNextRef();
    if (!ret) {
      ROS_ERROR_STREAM("[WrapperRosRBL]: Could not get next valid ref");
      return;
    }
    msg_ref.request.reference = ret.value();
  }

  if (!sc_set_ref_.call(msg_ref)) {
    ROS_ERROR("[WrapperRosRBL]: Failed to call service set reference");
  }
}// //}

void WrapperRosRBL::cbTmDiagnostics([[maybe_unused]] const ros::TimerEvent& te)// //{
{

  if (!is_initialized_) {
    return;
  }

  {
    std::scoped_lock lck(mtx_rbl_);

    pub_viz_target_.publish(getVizModGroupGoal(rbl_controller_->getGoal(), 0.5, _frame_));
    pub_viz_waypoint_.publish(getVizWaypoint(rbl_controller_->getWaypoint(), 0.5, _frame_));
    pub_viz_position_.publish(getVizPosition(rbl_controller_->getCurrentPosition(), 0.5, _frame_));
    pub_viz_centroid_.publish(getVizCentroid(rbl_controller_->getCentroid(), _frame_));
    pub_viz_cell_A_.publish(*getVizCellA(rbl_controller_->getCellA(), _frame_));
    pub_viz_inflated_map_.publish(*getVizInflatedMap(rbl_controller_->getInflatedMap(), _frame_));
    pub_viz_path_.publish(getVizPath(rbl_controller_->getPath(), _frame_));
  }
}// //}

bool WrapperRosRBL::cbSrvActivateControl([[maybe_unused]] std_srvs::Trigger::Request& req,// //{
                                         std_srvs::Trigger::Response&                 res)
{
  res.success = true;
  if (is_activated_) {
    res.message = "RBL is already active";
    ROS_WARN("[WrapperRosRBL]: %s", res.message.c_str());
  }
  else {
    res.message   = "RBL activated";
    is_activated_ = true;
    ROS_INFO("[WrapperRosRBL]: %s", res.message.c_str());
  }
  return true;
}// //}

bool WrapperRosRBL::cbSrvDeactivateControl([[maybe_unused]] std_srvs::Trigger::Request& req,// //{
                                           std_srvs::Trigger::Response&                 res)
{
  res.success = true;
  if (!is_activated_) {
    res.message = "RBL is already deactivated";
    ROS_WARN("[WrapperRosRBL]: %s", res.message.c_str());
  }
  else {
    res.message = "RBL deactivated";
    ROS_INFO("[WrapperRosRBL]: %s", res.message.c_str());
  }
  return true;
}// //}

bool WrapperRosRBL::cbSrvGotoPosition(mrs_msgs::Vec4::Request&  req,// //{
                                      mrs_msgs::Vec4::Response& res)
{
  {
    std::scoped_lock lck(mtx_rbl_);
    rbl_controller_->setGoal(Eigen::Vector3d{ req.goal[0], req.goal[1], req.goal[2] });
  }
  res.success = true;
  res.message = "Goal set";
  ROS_INFO("[WrapperRosRBL]: %s", res.message.c_str());
  return true;
}// //}

visualization_msgs::Marker WrapperRosRBL::getVizPosition(const Eigen::Vector3d& point,// //{
                                                         const double           scale,
                                                         const std::string&     frame)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id    = frame;
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "position";
  marker.id                 = 0;
  marker.type               = visualization_msgs::Marker::SPHERE;
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.position.x    = point(0);
  marker.pose.position.y    = point(1);
  marker.pose.position.z    = point(2);
  marker.pose.orientation.w = 1.0;
  marker.scale.x            = scale;
  marker.scale.y            = scale;
  marker.scale.z            = scale;
  marker.color.r            = 1.0;
  marker.color.g            = 0.0;
  marker.color.b            = 0.0;
  marker.color.a            = 0.3;

  return marker;
}// //}

visualization_msgs::Marker WrapperRosRBL::getVizModGroupGoal(const Eigen::Vector3d& point,// //{
                                                             const double           scale,
                                                             const std::string&     frame)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id    = frame;
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "modified-group-goal";
  marker.id                 = 0;
  marker.type               = visualization_msgs::Marker::SPHERE;
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.position.x    = point(0);
  marker.pose.position.y    = point(1);
  marker.pose.position.z    = point(2);
  marker.pose.orientation.w = 1.0;
  marker.scale.x            = scale;
  marker.scale.y            = scale;
  marker.scale.z            = scale;
  marker.color.r            = 0.0;
  marker.color.g            = 0.0;
  marker.color.b            = 1.0;
  marker.color.a            = 0.3;

  return marker;
}// //}

visualization_msgs::Marker WrapperRosRBL::getVizWaypoint(const Eigen::Vector3d& point,// //{
                                                         const double           scale,
                                                         const std::string&     frame)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id    = frame;
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "waypoint";
  marker.id                 = 0;
  marker.type               = visualization_msgs::Marker::SPHERE;
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.position.x    = point(0);
  marker.pose.position.y    = point(1);
  marker.pose.position.z    = point(2);
  marker.pose.orientation.w = 1.0;
  marker.scale.x            = scale;
  marker.scale.y            = scale;
  marker.scale.z            = scale;
  marker.color.r            = 0.0;
  marker.color.g            = 0.0;
  marker.color.b            = 1.0;
  marker.color.a            = 0.3;

  return marker;
}                                                           // //}

std::shared_ptr<sensor_msgs::PointCloud2> WrapperRosRBL::getVizCellA(const std::vector<Eigen::Vector3d>& points,// //{
                                                                     const std::string&                  frame)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

  pcl_cloud.points.resize(points.size());

  for (size_t i = 0; i < points.size(); ++i) {
    pcl_cloud.points[i].x = points[i].x();
    pcl_cloud.points[i].y = points[i].y();
    pcl_cloud.points[i].z = points[i].z();
  }

  auto ros_msg = std::make_shared<sensor_msgs::PointCloud2>();

  pcl::toROSMsg(pcl_cloud, *ros_msg);

  ros_msg->header.frame_id = frame;

  return ros_msg;
}// //}

std::shared_ptr<sensor_msgs::PointCloud2> WrapperRosRBL::getVizInflatedMap(const std::vector<Eigen::Vector3d>& points,// //{
                                                                           const std::string&                  frame)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

  pcl_cloud.points.resize(points.size());

  for (size_t i = 0; i < points.size(); ++i) {
    pcl_cloud.points[i].x = points[i].x();
    pcl_cloud.points[i].y = points[i].y();
    pcl_cloud.points[i].z = points[i].z();
  }

  auto ros_msg = std::make_shared<sensor_msgs::PointCloud2>();

  pcl::toROSMsg(pcl_cloud, *ros_msg);

  ros_msg->header.frame_id = frame;

  return ros_msg;
}// //}

nav_msgs::Path WrapperRosRBL::getVizPath(const std::vector<Eigen::Vector3d>& path,// //{
                                         const std::string&                  frame)
{
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = frame;

  for (const auto& pt : path) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = frame;
    pose.pose.position.x = pt.x();
    pose.pose.position.y = pt.y();
    pose.pose.position.z = pt.z();

    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    path_msg.poses.push_back(pose);
  }

  return path_msg;
}// //}

visualization_msgs::Marker WrapperRosRBL::getVizCentroid(const Eigen::Vector3d& point,// //{
                                                         const std::string&     frame)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id    = frame;
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "centroid";
  marker.id                 = 0;
  marker.type               = visualization_msgs::Marker::SPHERE;
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.position.x    = point(0);
  marker.pose.position.y    = point(1);
  marker.pose.position.z    = point(2);
  marker.pose.orientation.w = 1.0;
  marker.scale.x            = 0.2;
  marker.scale.y            = 0.2;
  marker.scale.z            = 0.2;
  marker.color.r            = 0.7;
  marker.color.g            = 0.5;
  marker.color.b            = 0.0;
  marker.color.a            = 1.0;

  return marker;
}// //}

Eigen::Vector3d WrapperRosRBL::pointToEigen(const geometry_msgs::Point& point)// //{
{
  return Eigen::Vector3d(point.x, point.y, point.z);
}// //}

Eigen::Vector3d WrapperRosRBL::vectorToEigen(const geometry_msgs::Vector3& vec)// //{
{
  return Eigen::Vector3d(vec.x, vec.y, vec.z);
}// //}

geometry_msgs::Point WrapperRosRBL::pointFromEigen(const Eigen::Vector3d& vec)// //{
{
  return createPoint(vec(0), vec(1), vec(2));
}// //}

geometry_msgs::Point WrapperRosRBL::createPoint(double x,// //{
                                                double y,
                                                double z)
{
  geometry_msgs::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}// //}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(WrapperRosRBL,
                       nodelet::Nodelet);
