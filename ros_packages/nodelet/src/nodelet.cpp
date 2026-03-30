// CUSTOM
#include "rbl_controller_core/rbl_controller.h"
#include <rbl_controller_node/msg/pose_velocity_array.hpp>

// MRS LIB
#include <mrs_lib/mutex.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/node.h>
#include <mrs_lib/service_server_handler.h>
#include <mrs_lib/service_client_handler.h>

// MRS MSGs
#include <mrs_msgs/msg/pose_with_covariance_array_stamped.hpp>
#include <mrs_msgs/msg/reference.hpp>
#include <mrs_msgs/msg/float64_stamped.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <mrs_msgs/srv/reference_stamped_srv.hpp>
#include <mrs_msgs/srv/vec4.hpp>
#include <mrs_msgs/srv/float64_srv.hpp>

// #include <mrs_msgs/msg/PoseWithCovarianceArrayStamped.hpp>
// #include <mrs_msgs/msg/Reference.hpp>
// #include <mrs_msgs/msg/ReferenceStampedSrv.hpp>
// #include <mrs_msgs/msg/Vec4.hpp>

// MSGS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// ROS 2
#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/trigger.hpp>

// OCTOMAP
#include <octomap/OcTree.h>

// EIGEN
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>


// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Standard CPP libs
#include <cmath>
#include <string>

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

namespace rbl_controller {

class WrapperRosRBL : public mrs_lib::Node {
public: 
  WrapperRosRBL(rclcpp::NodeOptions options);

  void initialize();

private:

  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;


  std::vector<State> group_states_;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> last_obstacle_cloud_;
  bool pcl_loaded_ = false;

  bool         is_initialized_ = false;
  bool         is_activated_   = false;

  bool        _group_odoms_enabled_ = false;
  bool        _add_agents_to_pcl_   = false;
  int         _group_odoms_size_    = 0;
  std::string _agent_name_;
  std::string _frame_;

  std::mutex                     mtx_rbl_;
  std::shared_ptr<RBLController> rbl_controller_;
  RBLParams                      rbl_params_;

  // | ----------------- sevice server callbacks ---------------- |

  bool cbSrvActivateControl(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger> srv_activate_control_;

  bool cbSrvDeactivateControl(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger> srv_deactivate_control_;

  // bool cbSrvGotoPosition(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  bool cbSrvGotoPosition(const std::shared_ptr<mrs_msgs::srv::Vec4::Request>  req,  const std::shared_ptr<mrs_msgs::srv::Vec4::Response> res);
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::Vec4> srv_goto_position_;

  bool cbSrvSetBetaD(const std::shared_ptr<mrs_msgs::srv::Float64Srv::Request> req, const std::shared_ptr<mrs_msgs::srv::Float64Srv::Response> res
  );
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::Float64Srv> srv_set_betaD_;

  // ros::ServiceServer srv_activate_control_;
  // bool               cbSrvActivateControl([[maybe_unused]] std_srvs::Trigger::Request& req,
  //                                         std_srvs::Trigger::Response&                 res);
  // ros::ServiceServer srv_deactivate_control_;
  // bool               cbSrvDeactivateControl([[maybe_unused]] std_srvs::Trigger::Request& req,
  //                                           std_srvs::Trigger::Response&                 res);
  // ros::ServiceServer srv_goto_position_;
  // bool               cbSrvGotoPosition(mrs_msgs::Vec4::Request&  req,
  //                                      mrs_msgs::Vec4::Response& res);


  // | --------------------- service clients -------------------- |

  mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv> sc_set_ref_;

  // ros::ServiceClient sc_set_ref_;

  // | --------------------- timer callbacks -------------------- |

  void cbTmSetRef();
  std::shared_ptr<TimerType> tm_set_ref_;

  void cbTmDiagnostics();
  std::shared_ptr<TimerType> tm_diagnostics_;
  // ros::Timer tm_set_ref_;
  // void       cbTmSetRef([[maybe_unused]] const ros::TimerEvent& te);
  // ros::Timer tm_diagnostics_;
  // void       cbTmDiagnostics([[maybe_unused]] const ros::TimerEvent& te);


  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<visualization_msgs::msg::Marker> pub_viz_position_;
  mrs_lib::PublisherHandler<visualization_msgs::msg::Marker> pub_viz_centroid_;
  mrs_lib::PublisherHandler<visualization_msgs::msg::Marker> pub_viz_seed_B_;
  mrs_lib::PublisherHandler<visualization_msgs::msg::Marker> pub_viz_target_;
  mrs_lib::PublisherHandler<visualization_msgs::msg::Marker> pub_viz_waypoint_;
  mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2> pub_viz_cell_A_;
  mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2> pub_viz_cell_A_sensed_;
  mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2> pub_viz_inflated_map_;
  mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2> pub_viz_cloud;
  mrs_lib::PublisherHandler<nav_msgs::msg::Path> pub_viz_path_;
  

  // ros::Publisher                            pub_viz_cell_A_;
  // ros::Publisher                            pub_viz_cell_A_sensed_;
  std::shared_ptr<sensor_msgs::msg::PointCloud2> getVizCellA(const std::vector<Eigen::Vector3d>& points,
                                                             const std::string&                  frame);
  // ros::Publisher                            pub_viz_inflated_map_;
  std::shared_ptr<sensor_msgs::msg::PointCloud2> getVizInflatedMap(const std::vector<Eigen::Vector3d>& points,
                                                                   const std::string&                  frame);
  // ros::Publisher                            pub_viz_cloud;
  std::shared_ptr<sensor_msgs::msg::PointCloud2> getVizPCL(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& pcl,  // //{
                                                           const std::string&                                     frame);
  // ros::Publisher             pub_viz_path_;
  std::shared_ptr<nav_msgs::msg::Path>             getVizPath(const std::vector<Eigen::Vector3d>& path,
                                                   const std::string&                  frame);
  // ros::Publisher             pub_viz_position_;
  visualization_msgs::msg::Marker getVizPosition(const Eigen::Vector3d& point,
                                            const double           scale,
                                            const std::string&     frame);
  // ros::Publisher             pub_viz_centroid_;
  // ros::Publisher             pub_viz_seed_B_;
  visualization_msgs::msg::Marker getVizCentroid(const Eigen::Vector3d& point,
                                            const std::string&     frame);
  // ros::Publisher             pub_viz_target_;
  visualization_msgs::msg::Marker getVizModGroupGoal(const Eigen::Vector3d& point,
                                                const double           scale,
                                                const std::string&     frame);
  // ros::Publisher             pub_viz_waypoint_;
  visualization_msgs::msg::Marker getVizWaypoint(const Eigen::Vector3d& point,
                                            const double           scale,
                                            const std::string&     frame);


  // | ----------------------- subscribers ---------------------- |
  bool octomap_msg_;
  mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>              sh_odom_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped>        sh_alt_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::PointCloud2>        sh_pcl_;
  mrs_lib::SubscriberHandler<octomap_msgs::msg::Octomap>           sh_octomap_;
  mrs_lib::SubscriberHandler<rbl_controller_node::msg::PoseVelocityArray> sh_group_states_;
  // std::vector<mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>> sh_group_odoms_;

  void updateGroupStates(const rbl_controller_node::msg::PoseVelocityArray::ConstSharedPtr& msg);



  std::shared_ptr<mrs_lib::Transformer> transformer_;

  Eigen::Vector3d                                 pointToEigen(const geometry_msgs::msg::Point& point);
  Eigen::Vector3d                                 vectorToEigen(const geometry_msgs::msg::Vector3& vec);
  geometry_msgs::msg::Point                            pointFromEigen(const Eigen::Vector3d& vec);
  geometry_msgs::msg::Point                            createPoint(double x,
                                                              double y,
                                                              double z);
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> addAgents2PCL(std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud,
                                                                const std::vector<State>& group_states,
                                                                const double              voxel_size,
                                                                const double              encumbrance);
};  // //}



WrapperRosRBL::WrapperRosRBL(rclcpp::NodeOptions options) : Node("wrapper_ros_rbl", options) {
  initialize();
}


void WrapperRosRBL::initialize()  // //{
{
  node_  = this->this_node_ptr();
  clock_ = node_->get_clock();

  cbkgrp_subs_   = this_node().create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_sc_     = this_node().create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_ss_     = this_node().create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_timers_ = this_node().create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);


  // ros::NodeHandle& nh = getPrivateNodeHandle();
  // NODELET_DEBUG("Initializing nodelet...");
  // ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(node_);

  param_loader.addYamlFileFromParam("config");
  param_loader.addYamlFileFromParam("custom_config");

  std::string odom_topic_name;
  double      rate_tm_set_ref;
  double      rate_tm_diagnostics;

  param_loader.loadParam("uav_name", _agent_name_);
  param_loader.loadParam("control_frame", _frame_);
  param_loader.loadParam("group_odoms/enable", _group_odoms_enabled_);
  param_loader.loadParam("group_odoms/add_to_pcl", _add_agents_to_pcl_);
  param_loader.loadParam("group_odoms/size", _group_odoms_size_);

  param_loader.loadParam("odometry_topic", odom_topic_name);
  param_loader.loadParam("rate/timer_set_ref", rate_tm_set_ref);
  param_loader.loadParam("rate/timer_diagnostics", rate_tm_diagnostics);

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
  param_loader.loadParam("rbl_controller/use_map", rbl_params_.use_map);
  param_loader.loadParam("rbl_controller/ciri", rbl_params_.ciri);
  param_loader.loadParam("rbl_controller/boundary_threshold", rbl_params_.boundary_threshold);
  param_loader.loadParam("rbl_controller/boundary_threshold_speed", rbl_params_.boundary_threshold_speed);
  param_loader.loadParam("rbl_controller/lidar_tilt", rbl_params_.lidar_tilt);
  param_loader.loadParam("rbl_controller/lidar_fov", rbl_params_.lidar_fov);
  param_loader.loadParam("rbl_controller/move_centroid_to_sensed_cell", rbl_params_.move_centroid_to_sensed_cell);
  param_loader.loadParam("rbl_controller/octomap/octomap_msg", octomap_msg_);
  param_loader.loadParam("rbl_controller/pcl/downsample", rbl_params_.downsample_pcl);
  param_loader.loadParam("rbl_controller/pcl/voxel_size", rbl_params_.voxel_size);
  param_loader.loadParam("rbl_controller/add_estimates_as_voxels", rbl_params_.add_estimates_as_voxels);
  param_loader.loadParam("replanner/inflation_bonus", rbl_params_.inflation_bonus);

  // rbl_params_.voxel_size = 2 * std::sqrt(rbl_params_.encumbrance / std::sqrt(3.0));

  if (!param_loader.loadedSuccessfully()) {
    // ROS_ERROR("[WrapperRosRBL]: Could not load all parameters!");
    RCLCPP_ERROR(node_->get_logger(), "failed to load non-optional parameters!");
    rclcpp::shutdown();
  }

  // mrs_lib::SubscriberHandlerOptions shopts;
  // shopts.node                                 = node_;
  // shopts.node_name                            = "WrapperRosRBL";
  // // shopts.no_message_timeout = mrs_lib::no_timeout;
  // shopts.threadsafe                           = true;
  // shopts.autostart                            = true;
  // shopts.subscription_options.callback_group  = cbkgrp_subs_;
  // shopts.queue_size         = 10;
  // shopts.transport_hints    = ros::TransportHints().tcpNoDelay();


  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandlerOptions shopts;

  shopts.node                                = node_;
  shopts.node_name                           = "WrapperRosRBL";
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  sh_odom_            = mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(shopts, "~/odom_in");
  sh_alt_             = mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped>(shopts, "~/alt_in");
  if (octomap_msg_){
    sh_octomap_          = mrs_lib::SubscriberHandler<octomap_msgs::msg::Octomap>(shopts, "~/octomap_in");
  } else {
    sh_pcl_             = mrs_lib::SubscriberHandler<sensor_msgs::msg::PointCloud2>(shopts, "~/pcl_in");
  }
  sh_group_states_    = mrs_lib::SubscriberHandler<rbl_controller_node::msg::PoseVelocityArray>(shopts, "~/group_states_in");
  
  // sh_group_odoms_     = std::vector<mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>>;
  // sh_group_odoms_     = std::vector<mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry> (shopts, "~/group_odoms_in")>;
  // sh_group_odoms_     = std::vector<mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>>(shopts, "~/group_odoms_in");


  // if (_group_odoms_enabled_) {

  //   do {
  //     ROS_INFO("[WrapperRosRBL]: Clearing group odom topics");
  //     sh_group_odoms_.clear();
  //     ros::master::V_TopicInfo all_topics;
  //     ros::master::getTopics(all_topics);

  //     for (const auto& topic : all_topics) {
  //       if (topic.name.find(_agent_name_ + "/") == std::string::npos &&
  //         topic.name.find(odom_topic_name) != std::string::npos) {
  //         RCLCPP_INFO(node_->get_logger(),"Subscribing to topic: %s",topic.c_str());
  //         sh_group_odoms_.push_back(mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, topic.name.c_str()));
  //       }
  //     }

  //     if (sh_group_odoms_.empty()) {
  //       ROS_WARN("[WrapperRosRBL]: No topics matched: %s", odom_topic_name.c_str());
  //     }
  //   } while (sh_group_odoms_.size() < _group_odoms_size_);
  // }
  


  // mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>              sh_odom_;
  // mrs_lib::SubscriberHandler<sensor_msgs::msg::Range>              sh_alt_;
  // mrs_lib::SubscriberHandler<sensor_msgs::msg::PointCloud2>        sh_pcl_;
  // std::vector<mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>> sh_group_odoms_;



  // sh_odom_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "odom_in");
  // sh_alt_  = mrs_lib::SubscribeHandler<sensor_msgs::Range>(shopts, "alt_in");
  // sh_pcl_  = mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>(shopts, "pcl_in");

  // | ------------------------- timers ------------------------- |

  mrs_lib::TimerHandlerOptions opts_autostart;

  opts_autostart.node           = node_;
  opts_autostart.autostart      = true;
  opts_autostart.callback_group = cbkgrp_timers_;

  {
    std::function<void()> callback_fn = std::bind(&WrapperRosRBL::cbTmSetRef, this);
    tm_set_ref_   = std::make_shared<TimerType>(opts_autostart, rclcpp::Rate(rate_tm_set_ref, clock_), callback_fn);
  }

  {
    std::function<void()> callback_fn = std::bind(&WrapperRosRBL::cbTmDiagnostics, this);
    tm_diagnostics_          = std::make_shared<TimerType>(opts_autostart, rclcpp::Rate(rate_tm_diagnostics, clock_), callback_fn);
  }

  // tm_set_ref_     = nh.createTimer(ros::Rate(rate_tm_set_ref), &WrapperRosRBL::cbTmSetRef, this);
  // tm_diagnostics_ = nh.createTimer(ros::Rate(rate_tm_diagnostics), &WrapperRosRBL::cbTmDiagnostics, this);


  // | --------------------- service servers -------------------- |

  srv_activate_control_ = mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_, "~/control_activation_in", std::bind(&WrapperRosRBL::cbSrvActivateControl, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  
  srv_deactivate_control_ = mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_, "~/control_deactivation_in", std::bind(&WrapperRosRBL::cbSrvDeactivateControl, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);

  srv_goto_position_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::Vec4>(
      node_, "~/goto_out", std::bind(&WrapperRosRBL::cbSrvGotoPosition, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(), cbkgrp_ss_);
  srv_set_betaD_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::Float64Srv>(
      node_, "~/set_betaD", std::bind(&WrapperRosRBL::cbSrvSetBetaD, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::SystemDefaultsQoS(),
      cbkgrp_ss_);

  // srv_activate_control_ = nh.advertiseService("control_activation_in", &WrapperRosRBL::cbSrvActivateControl, this);
  // srv_goto_position_    = nh.advertiseService("goto_in", &WrapperRosRBL::cbSrvGotoPosition, this);
  // srv_deactivate_control_ = nh.advertiseService("control_deactivation_in", &WrapperRosRBL::cbSrvDeactivateControl, this);

  // | --------------------- service clients -------------------- |
  
  sc_set_ref_ = mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>(node_, "~/ref_out", cbkgrp_sc_);

  // sc_set_ref_ = nh.serviceClient<mrs_msgs::ReferenceStampedSrv>("ref_out");

  // | ----------------------- publishers ----------------------- |

  // TODO remap in in launch.py
  pub_viz_position_      = mrs_lib::PublisherHandler<visualization_msgs::msg::Marker>(node_, "~/position");
  pub_viz_centroid_      = mrs_lib::PublisherHandler<visualization_msgs::msg::Marker>(node_, "~/centroid");
  pub_viz_seed_B_        = mrs_lib::PublisherHandler<visualization_msgs::msg::Marker>(node_, "~/seed_B");
  pub_viz_target_        = mrs_lib::PublisherHandler<visualization_msgs::msg::Marker>(node_, "~/target");
  pub_viz_waypoint_      = mrs_lib::PublisherHandler<visualization_msgs::msg::Marker>(node_, "~/replanner_waypoint");
  pub_viz_cell_A_        = mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2>(node_, "~/cell_a");
  pub_viz_cell_A_sensed_ = mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2>(node_, "~/actively_sensed_A");
  pub_viz_inflated_map_  = mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2>(node_, "~/inflated_map");
  pub_viz_cloud          = mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2>(node_, "~/cloud");
  pub_viz_path_          = mrs_lib::PublisherHandler<nav_msgs::msg::Path>(node_, "~/path");
  
  transformer_ = std::make_shared<mrs_lib::Transformer>(node_);
  // transformer_ = std::make_shared<mrs_lib::Transformer>(node_, "WrapperRosRBL");
  transformer_->retryLookupNewest(true);

  {
    std::scoped_lock lck(mtx_rbl_);
    rbl_controller_ = std::make_shared<RBLController>(rbl_params_);
    RCLCPP_INFO_ONCE(node_->get_logger(), "Initialized RBLController with params");
  }

  is_initialized_ = true;
  RCLCPP_INFO_ONCE(node_->get_logger(), "Initialization completed");
}  // //}

void WrapperRosRBL::updateGroupStates(const rbl_controller_node::msg::PoseVelocityArray::ConstSharedPtr& msg) {
  if (!msg) {
    RCLCPP_WARN(node_->get_logger(), "Received empty group states message");
    return;
  }

  if (msg->poses.size() != msg->velocities.size()) {
    RCLCPP_WARN(
        node_->get_logger(),
        "Ignoring group states message with mismatched array sizes: poses=%zu velocities=%zu",
        msg->poses.size(), msg->velocities.size());
    return;
  }

  std::vector<State> updated_group_states;
  updated_group_states.reserve(msg->poses.size());

  for (std::size_t i = 0; i < msg->poses.size(); ++i) {
    geometry_msgs::msg::PointStamped point_msg;
    point_msg.header = msg->header;
    point_msg.point = msg->poses[i].position;

    auto transformed_point = transformer_->transformSingle(point_msg, _frame_);
    if (!transformed_point) {
      RCLCPP_WARN(
          node_->get_logger(),
          "Failed to transform group state position %zu from %s to %s",
          i,
          msg->header.frame_id.c_str(),
          _frame_.c_str());
      continue;
    }

    geometry_msgs::msg::Vector3Stamped velocity_msg;
    velocity_msg.header = msg->header;
    velocity_msg.vector = msg->velocities[i];

    auto transformed_velocity = transformer_->transformSingle(velocity_msg, _frame_);
    if (!transformed_velocity) {
      RCLCPP_WARN(
          node_->get_logger(),
          "Failed to transform group state velocity %zu from %s to %s",
          i,
          msg->header.frame_id.c_str(),
          _frame_.c_str());
      continue;
    }

    State state;
    state.position = pointToEigen(transformed_point->point);
    state.velocity = vectorToEigen(transformed_velocity->vector);
    updated_group_states.push_back(state);
  }

  group_states_ = std::move(updated_group_states);
  rbl_controller_->setGroupStates(group_states_);
}

void WrapperRosRBL::cbTmSetRef()  // //{
{
  if (!is_initialized_) {
    return;
  }

  if (!is_activated_) {
    RCLCPP_INFO_ONCE(node_->get_logger(), "Waiting for activation");
    return;
  }
  RCLCPP_INFO_ONCE(node_->get_logger(), "After activation");

  auto msg_ref = std::make_shared<mrs_msgs::srv::ReferenceStampedSrv::Request>();
  msg_ref->header.frame_id = _frame_;
  msg_ref->header.stamp    = clock_->now();

  // mrs_msgs::srv::ReferenceStampedSrv msg_ref;
  // msg_ref.request.header.frame_id = _frame_;
  // msg_ref.request.header.stamp    = ros::Time::now();

  {
    std::scoped_lock lck(mtx_rbl_);
    if (sh_odom_.newMsg()) {
      auto                        odom = sh_odom_.getMsg();
      geometry_msgs::msg::PointStamped tmp_pt;
      tmp_pt.header = odom->header;
      tmp_pt.point  = odom->pose.pose.position;
      auto res      = transformer_->transformSingle(tmp_pt, _frame_);

      if (!res) {
        RCLCPP_ERROR(node_->get_logger(), "Could not transform odometry msg to control frame.");
        return;
      }
      rbl_controller_->setCurrentPosition(pointToEigen(res.value().point));
      RCLCPP_INFO_ONCE(node_->get_logger(), "Setted cur position to rbl");

      geometry_msgs::msg::Vector3Stamped tmp_vel;
      tmp_vel.header = odom->header;
      tmp_vel.vector = odom->twist.twist.linear;
      auto vel_res   = transformer_->transformSingle(tmp_vel, _frame_);

      if (!vel_res) {
        RCLCPP_ERROR(node_->get_logger(), "Could not transform velocity to control frame.");
        return;
      }
      rbl_controller_->setCurrentVelocity(vectorToEigen(vel_res->vector));
      RCLCPP_INFO_ONCE(node_->get_logger(), "Setted velocity to rbl");
      Eigen::Vector3d euler;

      double q_x = odom->pose.pose.orientation.x;
      double q_y = odom->pose.pose.orientation.y;
      double q_z = odom->pose.pose.orientation.z;
      double q_w = odom->pose.pose.orientation.w;

      // Roll (X-axis rotation)
      double sinr_cosp = 2.0 * (q_w * q_x + q_y * q_z);
      double cosr_cosp = 1.0 - 2.0 * (q_x * q_x + q_y * q_y);
      euler.x()        = std::atan2(sinr_cosp, cosr_cosp);

      // Pitch (Y-axis rotation)
      double sinp = 2.0 * (q_w * q_y - q_z * q_x);
      if (std::abs(sinp) >= 1) {
        euler.y() = std::copysign(M_PI / 2, sinp);  // Use 90 degrees if out of range
      }
      else {
        euler.y() = std::asin(sinp);
      }

      // Yaw (Z-axis rotation)
      double siny_cosp = 2.0 * (q_w * q_z + q_x * q_y);
      double cosy_cosp = 1.0 - 2.0 * (q_y * q_y + q_z * q_z);
      euler.z()        = std::atan2(siny_cosp, cosy_cosp);

      rbl_controller_->setRollPitchYaw(euler);
      RCLCPP_INFO_ONCE(node_->get_logger(), "Setted rpy to rbl");
    }

    if (sh_alt_.newMsg()) {
      auto alt = sh_alt_.getMsg();
      rbl_controller_->setAltitude(alt->value);
      RCLCPP_INFO_ONCE(node_->get_logger(), "Setted cur altitude to rbl");
    }

    if (sh_group_states_.newMsg()) {
      updateGroupStates(sh_group_states_.getMsg());
      RCLCPP_INFO_ONCE(node_->get_logger(), "Updated group states");
    }


    // TODO fix this after completing the filter_uavs
    // if (!sh_group_odoms_.empty()) {
    //   // make sure the vector has the right size
    //   if (group_states_.size() != sh_group_odoms_.size())
    //     group_states_.resize(sh_group_odoms_.size());

    //   for (size_t i = 0; i < sh_group_odoms_.size(); ++i) {
    //     auto& tmp_sh = sh_group_odoms_[i];

    //     if (tmp_sh.newMsg()) {
    //       auto odom = tmp_sh.getMsg();

    //       State tmp_state;
    //       geometry_msgs::msg::PointStamped tmp_pt;
    //       tmp_pt.header = odom->header;
    //       tmp_pt.point = odom->pose.pose.position;

    //       auto res = transformer_->transformSingle(tmp_pt, _frame_);
    //       if (!res) {
    //         RCLCPP_ERROR(node_->get_logger(), "Could not transform odometry msg to control frame.");
    //         continue;
    //       }
    //       tmp_state.position = pointToEigen(res.value().point);

    //       geometry_msgs::Vector3Stamped tmp_vel;
    //       tmp_vel.header = odom->header;
    //       tmp_vel.vector = odom->twist.twist.linear;

    //       auto vel_res = transformer_->transformSingle(tmp_vel, _frame_);
    //       if (!vel_res) {
    //         RCLCPP_ERROR(node_->get_logger(), "Could not transform velocity msg to control frame.");
    //         continue;
    //       }
    //       tmp_state.velocity = vectorToEigen(vel_res->vector);

    //       group_states_[i] = tmp_state;  // update only this agent
    //     }
    //   }

      // rbl_controller_->setGroupStates(group_states_);  // always has last known states
  }



    //     // std::vector<State> group_states;
    //     if (!sh_group_odoms_.empty()) {

    //       for (auto& tmp_sh : sh_group_odoms_) {

    //         if (tmp_sh.newMsg()) {
    //           State                       tmp_state;
    //           auto                        odom = tmp_sh.getMsg();
    //           geometry_msgs::PointStamped tmp_pt;
    //           tmp_pt.header = odom->header;
    //           tmp_pt.point  = odom->pose.pose.position;

    //           auto res = transformer_->transformSingle(tmp_pt, _frame_);

    //           if (!res) {
    //             ROS_ERROR_THROTTLE(3.0, "[WrapperRosRBL]: Could not transform odometry msg to control frame.");
    //             return;
    //           }
    //           tmp_state.position = pointToEigen(res.value().point);

    //           geometry_msgs::Vector3Stamped tmp_vel;
    //           tmp_vel.header = odom->header;
    //           tmp_vel.vector = odom->twist.twist.linear;

    //           auto vel_res = transformer_->transformSingle(tmp_vel, _frame_);
    //           if (!vel_res) {
    //             ROS_ERROR_THROTTLE(3.0, "[WrapperRosRBL]: Could not transform velocity msg to control frame.");
    //             return;
    //           }
    //           tmp_state.velocity = vectorToEigen(vel_res->vector);
    //           group_states.emplace_back(tmp_state);
    //         }
    //       }
    //       rbl_controller_->setGroupStates(group_states);
    //     }

    // for (size_t i = 0; i < sh_group_odoms_.size(); ++i) {
    //     auto& tmp_sh = sh_group_odoms_[i];

    //     if (tmp_sh.hasMsg()) {
    //         auto odom = tmp_sh.getMsg();

    //         geometry_msgs::PointStamped tmp_pt;
    //         tmp_pt.header = odom->header;
    //         tmp_pt.point = odom->pose.pose.position;

    //         auto res = transformer_->transformSingle(tmp_pt, _frame_);
    //         if (!res) {
    //             ROS_ERROR_THROTTLE(3.0, "[WrapperRosRBL]: Could not transform odometry msg to control frame.");
    //             continue;
    //         }

    //         geometry_msgs::Vector3Stamped tmp_vel;
    //         tmp_vel.header = odom->header;
    //         tmp_vel.vector = odom->twist.twist.linear;

    //         auto vel_res = transformer_->transformSingle(tmp_vel, _frame_);
    //         if (!vel_res) {
    //             ROS_ERROR_THROTTLE(3.0, "[WrapperRosRBL]: Could not transform velocity msg to control frame.");
    //             continue;
    //         }

    //         State tmp_state;
    //         tmp_state.position = pointToEigen(res.value().point);
    //         tmp_state.velocity = vectorToEigen(vel_res->vector);

    //         // update persistent state
    //         if (group_states.size() <= i)
    //             group_states.resize(sh_group_odoms_.size());
    //         group_states[i] = tmp_state;
    //     }
    // }

    // now always use last known states
    // rbl_controller_->setGroupStates(group_states);

    // std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    // if (sh_pcl_.getNumPublishers() > 0 && sh_pcl_.newMsg()) {

    //   auto msg = sh_pcl_.getMsg();
    //   if (msg->header.frame_id != _frame_) {
    //     ROS_ERROR_STREAM("[WrapperRosRBL]: PCL msg is not in frame: " << _frame_.c_str());
    //     return;
    //   }

    //   if (msg) {
    //     pcl::PointCloud<pcl::PointXYZI> temp_cloud;
    //     pcl::fromROSMsg(*msg, temp_cloud);
    //     cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(temp_cloud);
    //   }
    // }

    // if (!pcl_loaded_ && sh_pcl_.newMsg()) {
    //   auto msg = sh_pcl_.getMsg();
    //   if (msg->header.frame_id != _frame_) {
    //     ROS_ERROR_STREAM("[WrapperRosRBL]: PCL msg is not in frame: " << _frame_.c_str());
    //     return;
    //   }
    //   rbl_controller_->setPCL(msg);
    //   pcl_loaded_ = true;
    // }


  if (octomap_msg_) {
    if (sh_octomap_.newMsg()) {
      auto msg = sh_octomap_.getMsg();
      if (!msg) {
        RCLCPP_WARN(node_->get_logger(), "Received empty octomap message");
        return;
      }

      std::unique_ptr<octomap::AbstractOcTree> octree_base;
      if (msg->binary) {
        octree_base.reset(octomap_msgs::binaryMsgToMap(*msg));
      } else {
        octree_base.reset(octomap_msgs::fullMsgToMap(*msg));
      }
      if (!octree_base) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to convert %s octomap message to OcTree",
                     msg->binary ? "binary" : "full");
        return;
      }

      auto* octree = dynamic_cast<octomap::OcTree*>(octree_base.get());
      if (!octree) {
        RCLCPP_ERROR(node_->get_logger(), "Converted octomap is not an OcTree");
        return;
      }

      auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
      cloud->header.frame_id = _frame_;
      cloud->points.reserve(octree->size());
      const bool needs_transform = msg->header.frame_id != _frame_;
      std::size_t total_leaf_count = 0;
      std::size_t occupied_leaf_count = 0;
      std::size_t transform_failed_count = 0;

      for (auto it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {
        ++total_leaf_count;

        if (!octree->isNodeOccupied(*it)) {
          continue;
        }
        ++occupied_leaf_count;

        geometry_msgs::msg::Point point_msg;
        point_msg.x = it.getX();
        point_msg.y = it.getY();
        point_msg.z = it.getZ();

        if (needs_transform) {
          geometry_msgs::msg::PointStamped point_stamped;
          point_stamped.header = msg->header;
          point_stamped.point = point_msg;

          auto transformed_point = transformer_->transformSingle(point_stamped, _frame_);
          if (!transformed_point) {
            ++transform_failed_count;
            RCLCPP_WARN(node_->get_logger(), "Failed to transform octomap point from %s to %s",
                        msg->header.frame_id.c_str(), _frame_.c_str());
            continue;
          }

          point_msg = transformed_point->point;
        }

        pcl::PointXYZI point;
        point.x = static_cast<float>(point_msg.x);
        point.y = static_cast<float>(point_msg.y);
        point.z = static_cast<float>(point_msg.z);
        point.intensity = 1.0f;
        cloud->points.push_back(point);
      }

      cloud->width = static_cast<std::uint32_t>(cloud->points.size());
      cloud->height = 1;
      cloud->is_dense = true;

      RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *clock_, 2000,
          "Octomap stats: frame=%s -> %s, resolution=%.3f, total_leafs=%zu, occupied_leafs=%zu, transformed_points=%zu, "
          "transform_failures=%zu",
          msg->header.frame_id.c_str(), _frame_.c_str(), octree->getResolution(), total_leaf_count, occupied_leaf_count,
          cloud->points.size(), transform_failed_count);

      if (cloud->empty()) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *clock_, 2000,
            "Converted octomap cloud is empty. total_leafs=%zu, occupied_leafs=%zu, transform_failures=%zu",
            total_leaf_count, occupied_leaf_count, transform_failed_count);
      }

      last_obstacle_cloud_ = cloud;
      pcl_loaded_ = true;
      rbl_controller_->setPCL1(last_obstacle_cloud_);
      RCLCPP_INFO_ONCE(node_->get_logger(), "Setted last pcl to rbl");
    }

  } else {
    if (sh_pcl_.newMsg()) {
      auto msg = sh_pcl_.getMsg();
      pcl::PointCloud<pcl::PointXYZI> tmp;
      pcl::fromROSMsg(*msg, tmp);
      last_obstacle_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(tmp);
      pcl_loaded_ = true;
      rbl_controller_->setPCL1(last_obstacle_cloud_);
      RCLCPP_INFO_ONCE(node_->get_logger(), "Setted last pcl to rbl");
    }
  }

  if (!last_obstacle_cloud_) {
    RCLCPP_WARN(node_->get_logger(), "Waiting for obstacle cloud");
    return;
  }

  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*last_obstacle_cloud_);

  if (_group_odoms_enabled_ && _add_agents_to_pcl_) {
    cloud = addAgents2PCL(cloud,
                          group_states_,
                          rbl_params_.voxel_size,
                          rbl_params_.encumbrance);
  }

  if (cloud->empty()) {
    RCLCPP_ERROR(node_->get_logger(), "PCL is empty");
    return;
  }

  rbl_controller_->setPCL(cloud);
  RCLCPP_INFO_ONCE(node_->get_logger(), "Setted curent pcl to rbl");
  pub_viz_cloud.publish(*getVizPCL(cloud, _frame_));

    // if (_group_odoms_enabled_ && _add_agents_to_pcl_) {
    //   cloud = addAgents2PCL(cloud, group_states, rbl_params_.voxel_size, rbl_params_.encumbrance);
    // }
    // if (cloud->empty()) {
    //   ROS_ERROR("[WrapperRosRBL]: PCL is empty");
    //   return;
    // }
    // rbl_controller_->setPCL(cloud);
    // pub_viz_cloud.publish(*getVizPCL(cloud, _frame_));

  auto ret = rbl_controller_->getNextRef();
  if (!ret) {
    RCLCPP_ERROR(node_->get_logger(), "Could not get next valid ref");
    return;
  }
  // msg_ref.request.reference = ret.value();
  msg_ref->reference = ret.value();
  
  // sc_set_ref_.callAsync(msg_ref,[this](auto res)
  //   {
  //     if (!res->success) {
  //       RCLCPP_WARN(node_->get_logger(), "Failed to call service");
  //     }
  //   }
  // );
  auto res = sc_set_ref_.callSync(msg_ref);

  if (!res) {
    RCLCPP_WARN(node_->get_logger(), "Failed to set reference");
  }

  // if (!sc_set_ref_.callSync(msg_ref)) {
  //   RCLCPP_ERROR(node_->get_logger(), "Failed to call service set reference");
  // }
}  // //}

void WrapperRosRBL::cbTmDiagnostics()  // //{
{

  if (!is_initialized_) {
    return;
  }

  Eigen::Vector3d goal;
  Eigen::Vector3d waypoint;
  Eigen::Vector3d current_position;
  Eigen::Vector3d centroid;
  Eigen::Vector3d seed_b;
  std::vector<Eigen::Vector3d> cell_a_points;
  std::vector<Eigen::Vector3d> sensed_cell_a_points;
  std::vector<Eigen::Vector3d> inflated_map_points;
  std::vector<Eigen::Vector3d> path_points;

  {
    std::scoped_lock lck(mtx_rbl_);
    goal                 = rbl_controller_->getGoal();
    waypoint             = rbl_controller_->getWaypoint();
    current_position     = rbl_controller_->getCurrentPosition();
    centroid             = rbl_controller_->getCentroid();
    seed_b               = rbl_controller_->getSeedB();
    cell_a_points        = rbl_controller_->getCellA();
    sensed_cell_a_points = rbl_controller_->getSensedCellA();
    inflated_map_points  = rbl_controller_->getInflatedMap();
    path_points          = rbl_controller_->getPath();
  }

  pub_viz_target_.publish(getVizModGroupGoal(goal, 2 * rbl_params_.encumbrance, _frame_));
  pub_viz_waypoint_.publish(getVizWaypoint(waypoint, 2 * rbl_params_.encumbrance, _frame_));
  pub_viz_position_.publish(getVizPosition(current_position, 2 * rbl_params_.encumbrance, _frame_));
  pub_viz_centroid_.publish(getVizCentroid(centroid, _frame_));
  pub_viz_seed_B_.publish(getVizCentroid(seed_b, _frame_));

  auto cell_A = getVizCellA(cell_a_points, _frame_);
  if (cell_A) {
    pub_viz_cell_A_.publish(*cell_A);
  } else {
    RCLCPP_WARN(node_->get_logger(), "Failed to publish cell A");
  }

  auto cell_A_sensed = getVizCellA(sensed_cell_a_points, _frame_);
  if (cell_A_sensed) {
    pub_viz_cell_A_sensed_.publish(*cell_A_sensed);
  } else {
    RCLCPP_WARN(node_->get_logger(), "Failed to publish sensed cell A");
  }

  auto inflated_map = getVizInflatedMap(inflated_map_points, _frame_);
  if (inflated_map) {
    pub_viz_inflated_map_.publish(*inflated_map);
  } else {
    RCLCPP_WARN(node_->get_logger(), "Failed to publish inflated map");
  }

  auto path = getVizPath(path_points, _frame_);
  if (path) {
    pub_viz_path_.publish(*path);
  } else {
    RCLCPP_WARN(node_->get_logger(), "Failed to publish planned path");
  }
}  // //}
bool cbSrvActivateControl(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> res);

bool WrapperRosRBL::cbSrvActivateControl([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req,  // //{
                                                         const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  res->success = true;
  if (is_activated_) {
    res->message = "RBL is already active";
    RCLCPP_WARN(node_->get_logger(), "%s", res->message.c_str());
  }
  else {
    res->message   = "RBL activated";
    is_activated_ = true;
    RCLCPP_INFO(node_->get_logger(), "%s", res->message.c_str());
  }
  return true;
}  // //}

bool WrapperRosRBL::cbSrvDeactivateControl([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req,  // //{
                                                            const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  res->success = true;
  if (!is_activated_) {
    res->message = "RBL is already deactivated";
    RCLCPP_WARN(node_->get_logger(), "%s", res->message.c_str());
  }
  else {
    res->message = "RBL deactivated";
    RCLCPP_INFO(node_->get_logger(), "%s", res->message.c_str());
  }
  return true;
}  // //}

bool WrapperRosRBL::cbSrvSetBetaD(
  const std::shared_ptr<mrs_msgs::srv::Float64Srv::Request> req,
  const std::shared_ptr<mrs_msgs::srv::Float64Srv::Response> res)
{
  {
    std::scoped_lock lck(mtx_rbl_);

    rbl_params_.betaD = req->value;

    rbl_controller_->setBetaD(req->value);
  }

  RCLCPP_INFO(node_->get_logger(), "betaD set to %.3f", req->value);

  res->success = true;
  res->message = "betaD updated";

  return true;
}

bool WrapperRosRBL::cbSrvGotoPosition(const std::shared_ptr<mrs_msgs::srv::Vec4::Request>  req,  // //{
                                      const std::shared_ptr<mrs_msgs::srv::Vec4::Response> res)
{
  {
    std::scoped_lock lck(mtx_rbl_);
    rbl_controller_->setGoal(Eigen::Vector3d{ req->goal[0], req->goal[1], req->goal[2] });
    RCLCPP_INFO(node_->get_logger(), "RBL goal set to [%.3f, %.3f, %.3f], heading %.3f",
                req->goal[0], req->goal[1], req->goal[2], req->goal[3]);
  }
  res->success = true;
  res->message = "Goal set";
  RCLCPP_INFO(node_->get_logger(), "%s", res->message.c_str());
  return true;
}  // //}

visualization_msgs::msg::Marker WrapperRosRBL::getVizPosition(const Eigen::Vector3d& point,  // //{
                                                         const double           scale,
                                                         const std::string&     frame)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id    = frame;
  marker.header.stamp       = clock_->now();
  marker.ns                 = "position";
  marker.id                 = 0;
  marker.type               = visualization_msgs::msg::Marker::SPHERE;
  marker.action             = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x    = point.x();
  marker.pose.position.y    = point.y();
  marker.pose.position.z    = point.z();
  marker.pose.orientation.w = 1.0;
  marker.scale.x            = scale;
  marker.scale.y            = scale;
  marker.scale.z            = scale;
  marker.color.r            = 1.0;
  marker.color.g            = 0.0;
  marker.color.b            = 0.0;
  marker.color.a            = 0.3;

  return marker;
}  // //}

visualization_msgs::msg::Marker WrapperRosRBL::getVizModGroupGoal(const Eigen::Vector3d& point,  // //{
                                                                  const double           scale,
                                                                  const std::string&     frame)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id    = frame;
  marker.header.stamp       = clock_->now();
  marker.ns                 = "modified-group-goal";
  marker.id                 = 0;
  marker.type               = visualization_msgs::msg::Marker::SPHERE;
  marker.action             = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x    = point.x();
  marker.pose.position.y    = point.y();
  marker.pose.position.z    = point.z();
  marker.pose.orientation.w = 1.0;
  marker.scale.x            = scale;
  marker.scale.y            = scale;
  marker.scale.z            = scale;
  marker.color.r            = 0.0;
  marker.color.g            = 0.0;
  marker.color.b            = 1.0;
  marker.color.a            = 0.3;

  return marker;
}  // //}

visualization_msgs::msg::Marker WrapperRosRBL::getVizWaypoint(const Eigen::Vector3d& point,  // //{
                                                              const double           scale,
                                                              const std::string&     frame)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id    = frame;
  marker.header.stamp       = clock_->now();
  marker.ns                 = "waypoint";
  marker.id                 = 0;
  marker.type               = visualization_msgs::msg::Marker::SPHERE;
  marker.action             = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x    = point.x();
  marker.pose.position.y    = point.y();
  marker.pose.position.z    = point.z();
  marker.pose.orientation.w = 1.0;
  marker.scale.x            = scale;
  marker.scale.y            = scale;
  marker.scale.z            = scale;
  marker.color.r            = 0.0;
  marker.color.g            = 0.0;
  marker.color.b            = 1.0;
  marker.color.a            = 0.3;

  return marker;
}  // //}

std::shared_ptr<sensor_msgs::msg::PointCloud2> WrapperRosRBL::getVizCellA(const std::vector<Eigen::Vector3d>& points,  // //{
                                                                          const std::string&                  frame)
{
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud;

  pcl_cloud.points.resize(points.size());
  pcl_cloud.width = pcl_cloud.points.size();
  pcl_cloud.height = 1;
  pcl_cloud.is_dense = true;

  for (size_t i = 0; i < points.size(); ++i) {
    pcl_cloud.points[i].x = points[i].x();
    pcl_cloud.points[i].y = points[i].y();
    pcl_cloud.points[i].z = points[i].z();
  }

  auto ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

  pcl::toROSMsg(pcl_cloud, *ros_msg);

  ros_msg->header.frame_id = frame;
  ros_msg->header.stamp = clock_->now();

  return ros_msg;
}  // //}

std::shared_ptr<sensor_msgs::msg::PointCloud2>
WrapperRosRBL::getVizInflatedMap(const std::vector<Eigen::Vector3d>& points,  // //{
                                 const std::string&                  frame)
{
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud;

  pcl_cloud.points.resize(points.size());
  pcl_cloud.width = pcl_cloud.points.size();
  pcl_cloud.height = 1;
  pcl_cloud.is_dense = true;

  for (size_t i = 0; i < points.size(); ++i) {
    pcl_cloud.points[i].x = points[i].x();
    pcl_cloud.points[i].y = points[i].y();
    pcl_cloud.points[i].z = points[i].z();
  }

  auto ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

  pcl::toROSMsg(pcl_cloud, *ros_msg);

  ros_msg->header.frame_id = frame;
  ros_msg->header.stamp = clock_->now();

  return ros_msg;
}  // //}

std::shared_ptr<sensor_msgs::msg::PointCloud2>
WrapperRosRBL::getVizPCL(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& pcl,  // //{
                         const std::string&                                     frame)
{
  auto ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

  pcl::toROSMsg(*pcl, *ros_msg);

  ros_msg->header.frame_id = frame;
  ros_msg->header.stamp = clock_->now();

  return ros_msg;
}  // //}

std::shared_ptr<nav_msgs::msg::Path> WrapperRosRBL::getVizPath(const std::vector<Eigen::Vector3d>& path,  // //{
                                                               const std::string&                  frame)
{
  auto path_msg = std::make_shared<nav_msgs::msg::Path>();
  path_msg->header.stamp    = clock_->now();
  path_msg->header.frame_id = frame;

  for (const auto& pt : path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp    = clock_->now();
    pose.header.frame_id = frame;
    pose.pose.position.x = pt.x();
    pose.pose.position.y = pt.y();
    pose.pose.position.z = pt.z();

    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    path_msg->poses.push_back(pose);
  }

  return path_msg;
}  // //}

visualization_msgs::msg::Marker WrapperRosRBL::getVizCentroid(const Eigen::Vector3d& point,  // //{
                                                              const std::string&     frame)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id    = frame;
  marker.header.stamp       = clock_->now();
  marker.ns                 = "centroid";
  marker.id                 = 0;
  marker.type               = visualization_msgs::msg::Marker::SPHERE;
  marker.action             = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x    = point.x();
  marker.pose.position.y    = point.y();
  marker.pose.position.z    = point.z();
  marker.pose.orientation.w = 1.0;
  marker.scale.x            = 0.2;
  marker.scale.y            = 0.2;
  marker.scale.z            = 0.2;
  marker.color.r            = 0.7;
  marker.color.g            = 0.5;
  marker.color.b            = 0.0;
  marker.color.a            = 1.0;

  return marker;
}  // //}

Eigen::Vector3d WrapperRosRBL::pointToEigen(const geometry_msgs::msg::Point& point)  // //{
{
  return Eigen::Vector3d(point.x, point.y, point.z);
}  // //}

Eigen::Vector3d WrapperRosRBL::vectorToEigen(const geometry_msgs::msg::Vector3& vec)  // //{
{
  return Eigen::Vector3d(vec.x, vec.y, vec.z);
}  // //}

geometry_msgs::msg::Point WrapperRosRBL::pointFromEigen(const Eigen::Vector3d& vec)  // //{
{
  return createPoint(vec(0), vec(1), vec(2));
}  // //}

geometry_msgs::msg::Point WrapperRosRBL::createPoint(double x,  // //{
                                                double y,
                                                double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}  // //}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>
WrapperRosRBL::addAgents2PCL(std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud,
                             const std::vector<State>&                        group_states,
                             const double                                     voxel_size,
                             const double                                     encumbrance)
{
  const int num_voxels_half_side = std::ceil(encumbrance / voxel_size);
  // injected_points_map_.clear();
  //

  for (const auto& state : group_states) {
    const Eigen::Vector3d& position = state.position;
    RCLCPP_ERROR(node_->get_logger(), "Adding points to PCL at: %.2f, %.2f, %.2f", position.x(), position.y(), position.z());

    const int center_nx = std::floor(position.x() / voxel_size);
    const int center_ny = std::floor(position.y() / voxel_size);
    const int center_nz = std::floor(position.z() / voxel_size);

    for (int dx = -num_voxels_half_side; dx <= num_voxels_half_side; ++dx) {
      for (int dy = -num_voxels_half_side; dy <= num_voxels_half_side; ++dy) {
        for (int dz = -num_voxels_half_side; dz <= num_voxels_half_side; ++dz) {

          double dist = std::sqrt(dx * dx + dy * dy + dz * dz) * voxel_size;

          if (dist > encumbrance + voxel_size)
            continue;

          int current_nx = center_nx + dx;
          int current_ny = center_ny + dy;
          int current_nz = center_nz + dz;

          double voxel_center_x = (current_nx + 0.5) * voxel_size;
          double voxel_center_y = (current_ny + 0.5) * voxel_size;
          double voxel_center_z = (current_nz + 0.5) * voxel_size;
          
          pcl::PointXYZI pt;
          pt.x = voxel_center_x;
          pt.y = voxel_center_y;
          pt.z = voxel_center_z;
          pt.intensity = 1.0f;

          cloud->push_back(pt);

          // cloud->push_back(pcl::PointXYZI(voxel_center_x, voxel_center_y, voxel_center_z, 1.0f));
        }
      }
    }
    // for (int dx = -num_voxels_half_side; dx <= num_voxels_half_side; ++dx) {
    //   for (int dy = -num_voxels_half_side; dy <= num_voxels_half_side; ++dy) {
    //     for (int dz = -num_voxels_half_side; dz <= num_voxels_half_side; ++dz) {
    //       int current_nx = center_nx + dx;
    //       int current_ny = center_ny + dy;
    //       int current_nz = center_nz + dz;

    //       double        voxel_center_x = (current_nx + 0.5) * voxel_size;
    //       double        voxel_center_y = (current_ny + 0.5) * voxel_size;
    //       double        voxel_center_z = (current_nz + 0.5) * voxel_size;
    //       pcl::PointXYZI p(voxel_center_x, voxel_center_y, voxel_center_z);
    //       cloud->push_back(p);
    //       Eigen::Vector3d point = Eigen::Vector3d(voxel_center_x, voxel_center_y, voxel_center_z);
    //       // injected_points_map_.push_back(point);
    //     }
    //   }
    // }
  }
  return cloud;
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rbl_controller::WrapperRosRBL);
