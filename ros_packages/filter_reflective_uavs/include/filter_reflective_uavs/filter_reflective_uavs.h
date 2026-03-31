#ifndef FILTER_REFLECTIVE_UAVS_FILTER_REFLECTIVE_UAVS_H
#define FILTER_REFLECTIVE_UAVS_FILTER_REFLECTIVE_UAVS_H

#include <memory>
#include <queue>
#include <vector>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace filter_reflective_uavs {

struct FilterParams {
  double min_intensity = 250.0;
  double max_intensity = 255.0;
  double search_radius = 1.0;
  double max_distance_from_seed = 1.0;
  int max_removed_points = 200;
  bool use_voxel_grid = true;
  double voxel_grid_size_x = 0.04;
  double voxel_grid_size_y = 0.04;
  double voxel_grid_size_z = 0.04;
  double dt = 0.2;
  double max_no_update = 1.0;
  double gate_treshold = 2.0;
  double reflective_clustering_tolerance = 0.4;
  int reflective_clustering_min_points = 1;
  int reflective_clustering_max_points = 999999;
  bool filter_out_myself_enabled = true;
  double filter_out_myself_dist = 1.0;
  bool load_gt_uav_positions = false;
  bool debug = false;
  double time_keep = 0.2;
};

struct Detection {
  double stamp_sec = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
};

struct Track {
  int id = 0;
  double last_update_sec = 0.0;
  Eigen::Matrix<double, 6, 1> x = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 6> P = Eigen::Matrix<double, 6, 6>::Identity();
};

struct FilterOutput {
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> filtered_cloud;
  std::vector<Detection> reflective_centroids;
  std::vector<Track> tracks;
};

class FilterReflectiveUavs {
public:
  explicit FilterReflectiveUavs(FilterParams params);

  void setAgentPosition(const Eigen::Vector3d& position);
  void setInjectedUavPositions(const std::vector<Eigen::Vector3d>& positions);

  FilterOutput processPointCloud(
      const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>>& cloud,
      double stamp_sec);

private:
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> injectGtUavPositions(
      const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>>& cloud) const;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> extractReflectivePoints(
      const pcl::PointCloud<pcl::PointXYZI>& cloud) const;
  std::vector<Detection> clusterToCentroids(
      const std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud,
      double stamp_sec) const;
  std::vector<Detection> filterLatestDetections(
      const std::vector<Detection>& detections) const;
  void updateTracks(const std::vector<Detection>& detections, double stamp_sec);
  void predictTrack(Track& track, double stamp_sec) const;
  void updateTrack(Track& track, const Eigen::Vector3d& measurement, double stamp_sec) const;
  void initializeTrack(const Eigen::Vector3d& measurement, double stamp_sec);
  void deleteStaleTracks(double stamp_sec);
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> removeTrackedUavs(
      const pcl::PointCloud<pcl::PointXYZI>& cloud) const;

  FilterParams params_;
  int next_track_id_ = 0;
  Eigen::Vector3d agent_pos_ = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> injected_uav_positions_;
  std::vector<Detection> collected_centroid_positions_;
  std::vector<Track> tracks_;
};

}  // namespace filter_reflective_uavs

#endif
