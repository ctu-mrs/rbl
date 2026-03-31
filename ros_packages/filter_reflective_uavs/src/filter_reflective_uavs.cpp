#include "filter_reflective_uavs/filter_reflective_uavs.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <utility>

#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace filter_reflective_uavs {

namespace {

double distanceSquared(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  return (a - b).squaredNorm();
}

}  // namespace

FilterReflectiveUavs::FilterReflectiveUavs(FilterParams params) : params_(std::move(params))  // //{
{
}  // //}

void FilterReflectiveUavs::setAgentPosition(const Eigen::Vector3d& position)  // //{
{
  agent_pos_ = position;
}  // //}

void FilterReflectiveUavs::setInjectedUavPositions(const std::vector<Eigen::Vector3d>& positions)  // //{
{
  injected_uav_positions_ = positions;
}  // //}

FilterOutput FilterReflectiveUavs::processPointCloud(  // //{
    const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>>& cloud,
    double                                                         stamp_sec)
{
  FilterOutput output;
  output.filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  if (!cloud) {
    return output;
  }

  auto cloud_with_injected_positions = injectGtUavPositions(cloud);
  auto reflective_cloud              = extractReflectivePoints(*cloud_with_injected_positions);
  auto detections                    = clusterToCentroids(reflective_cloud, stamp_sec);

  for (const auto& detection : detections) {
    collected_centroid_positions_.push_back(detection);
  }

  collected_centroid_positions_.erase(
      std::remove_if(
          collected_centroid_positions_.begin(),
          collected_centroid_positions_.end(),
          [this, stamp_sec](const Detection& detection) {
            return stamp_sec - detection.stamp_sec > params_.time_keep;
          }),
      collected_centroid_positions_.end());

  output.reflective_centroids = filterLatestDetections(collected_centroid_positions_);
  updateTracks(output.reflective_centroids, stamp_sec);
  output.tracks         = tracks_;
  output.filtered_cloud = removeTrackedUavs(*cloud_with_injected_positions);
  return output;
}  // //}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> FilterReflectiveUavs::injectGtUavPositions(
    const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>>& cloud) const  // //{
{
  auto cloud_with_injected_positions = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  if (cloud) {
    *cloud_with_injected_positions = *cloud;
  }

  if (!params_.load_gt_uav_positions) {
    return cloud_with_injected_positions;
  }

  for (const auto& position : injected_uav_positions_) {
    pcl::PointXYZI point;
    point.x         = static_cast<float>(position.x());
    point.y         = static_cast<float>(position.y());
    point.z         = static_cast<float>(position.z());
    point.intensity = static_cast<float>(params_.max_intensity);
    cloud_with_injected_positions->points.push_back(point);
  }

  cloud_with_injected_positions->width =
      static_cast<std::uint32_t>(cloud_with_injected_positions->points.size());
  cloud_with_injected_positions->height   = 1;
  cloud_with_injected_positions->is_dense = false;
  return cloud_with_injected_positions;
}  // //}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> FilterReflectiveUavs::extractReflectivePoints(
    const pcl::PointCloud<pcl::PointXYZI>& cloud) const  // //{
{
  auto reflective_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  reflective_cloud->points.reserve(cloud.points.size());

  for (const auto& point : cloud.points) {
    if (point.intensity < params_.min_intensity || point.intensity > params_.max_intensity) {
      continue;
    }
    reflective_cloud->points.push_back(point);
  }

  reflective_cloud->width    = static_cast<std::uint32_t>(reflective_cloud->points.size());
  reflective_cloud->height   = 1;
  reflective_cloud->is_dense = false;

  if (!params_.use_voxel_grid || reflective_cloud->empty()) {
    return reflective_cloud;
  }

  auto downsampled = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setInputCloud(reflective_cloud);
  voxel_grid.setLeafSize(
      static_cast<float>(params_.voxel_grid_size_x),
      static_cast<float>(params_.voxel_grid_size_y),
      static_cast<float>(params_.voxel_grid_size_z));
  voxel_grid.filter(*downsampled);
  return downsampled;
}  // //}

std::vector<Detection> FilterReflectiveUavs::clusterToCentroids(  // //{
    const std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud,
    double                                                   stamp_sec) const
{
  std::vector<Detection> detections;
  if (!cloud || cloud->empty()) {
    return detections;
  }

  auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZI>>();
  tree->setInputCloud(cloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZI> cluster_extraction;
  cluster_extraction.setClusterTolerance(static_cast<float>(params_.reflective_clustering_tolerance));
  cluster_extraction.setMinClusterSize(params_.reflective_clustering_min_points);
  cluster_extraction.setMaxClusterSize(params_.reflective_clustering_max_points);
  cluster_extraction.setSearchMethod(tree);
  cluster_extraction.setInputCloud(cloud);

  std::vector<pcl::PointIndices> clusters;
  cluster_extraction.extract(clusters);

  detections.reserve(clusters.size());
  for (const auto& cluster : clusters) {
    Eigen::Vector4f centroid = Eigen::Vector4f::Zero();
    pcl::compute3DCentroid(*cloud, cluster.indices, centroid);
    detections.push_back(Detection{stamp_sec, centroid.head<3>().cast<double>()});
  }

  return detections;
}  // //}

std::vector<Detection> FilterReflectiveUavs::filterLatestDetections(
    const std::vector<Detection>& detections) const  // //{
{
  std::vector<Detection> filtered;
  if (detections.empty()) {
    return filtered;
  }

  std::vector<bool> consumed(detections.size(), false);
  const double      radius_sq = params_.search_radius * params_.search_radius;

  for (std::size_t i = 0; i < detections.size(); ++i) {
    if (consumed[i]) {
      continue;
    }

    Detection newest = detections[i];
    consumed[i]      = true;

    for (std::size_t j = i + 1; j < detections.size(); ++j) {
      if (consumed[j]) {
        continue;
      }
      if (distanceSquared(detections[i].position, detections[j].position) > radius_sq) {
        continue;
      }

      consumed[j] = true;
      if (detections[j].stamp_sec > newest.stamp_sec) {
        newest = detections[j];
      }
    }

    filtered.push_back(newest);
  }

  return filtered;
}  // //}

void FilterReflectiveUavs::updateTracks(  // //{
    const std::vector<Detection>& detections,
    double                        stamp_sec)
{
  for (auto& track : tracks_) {
    predictTrack(track, stamp_sec);
  }

  struct Candidate {
    std::size_t track_index;
    std::size_t detection_index;
    double      distance_sq;
  };

  std::vector<Candidate> candidates;
  const double           gate_sq = params_.gate_treshold * params_.gate_treshold;

  for (std::size_t track_index = 0; track_index < tracks_.size(); ++track_index) {
    const auto predicted_position = tracks_[track_index].x.head<3>();
    for (std::size_t detection_index = 0; detection_index < detections.size(); ++detection_index) {
      const double dist_sq = distanceSquared(predicted_position, detections[detection_index].position);
      if (dist_sq <= gate_sq) {
        candidates.push_back(Candidate{track_index, detection_index, dist_sq});
      }
    }
  }

  std::sort(
      candidates.begin(),
      candidates.end(),
      [](const Candidate& lhs, const Candidate& rhs) {
        return lhs.distance_sq < rhs.distance_sq;
      });

  std::vector<bool> assigned_tracks(tracks_.size(), false);
  std::vector<bool> assigned_detections(detections.size(), false);

  for (const auto& candidate : candidates) {
    if (assigned_tracks[candidate.track_index] || assigned_detections[candidate.detection_index]) {
      continue;
    }

    updateTrack(tracks_[candidate.track_index], detections[candidate.detection_index].position, stamp_sec);
    assigned_tracks[candidate.track_index]         = true;
    assigned_detections[candidate.detection_index] = true;
  }

  for (std::size_t detection_index = 0; detection_index < detections.size(); ++detection_index) {
    if (!assigned_detections[detection_index]) {
      initializeTrack(detections[detection_index].position, stamp_sec);
    }
  }

  deleteStaleTracks(stamp_sec);
}  // //}

void FilterReflectiveUavs::predictTrack(Track& track, double stamp_sec) const  // //{
{
  const double dt = std::max(0.0, stamp_sec - track.last_update_sec);
  if (dt <= 0.0) {
    return;
  }

  Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
  F(0, 3) = dt;
  F(1, 4) = dt;
  F(2, 5) = dt;

  Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
  const double                 process_noise = std::max(params_.dt, 1e-3);
  Q.diagonal() << process_noise, process_noise, process_noise, 1.0, 1.0, 1.0;

  track.x               = F * track.x;
  track.P               = F * track.P * F.transpose() + Q;
  track.last_update_sec = stamp_sec;
}  // //}

void FilterReflectiveUavs::updateTrack(  // //{
    Track&                 track,
    const Eigen::Vector3d& measurement,
    double                 stamp_sec) const
{
  Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;
  H(2, 2) = 1.0;

  Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 0.05;

  const Eigen::Vector3d           innovation = measurement - H * track.x;
  const Eigen::Matrix3d           S          = H * track.P * H.transpose() + R;
  const Eigen::Matrix<double, 6, 3> K        = track.P * H.transpose() * S.inverse();

  track.x               = track.x + K * innovation;
  track.P               = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * track.P;
  track.last_update_sec = stamp_sec;
}  // //}

void FilterReflectiveUavs::initializeTrack(  // //{
    const Eigen::Vector3d& measurement,
    double                 stamp_sec)
{
  Track track;
  track.id              = next_track_id_++;
  track.last_update_sec = stamp_sec;
  track.x.head<3>()     = measurement;
  track.x.tail<3>().setZero();
  track.P = Eigen::Matrix<double, 6, 6>::Identity();
  tracks_.push_back(track);
}  // //}

void FilterReflectiveUavs::deleteStaleTracks(double stamp_sec)  // //{
{
  tracks_.erase(
      std::remove_if(
          tracks_.begin(),
          tracks_.end(),
          [this, stamp_sec](const Track& track) {
            return stamp_sec - track.last_update_sec > params_.max_no_update;
          }),
      tracks_.end());
}  // //}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> FilterReflectiveUavs::removeTrackedUavs(
    const pcl::PointCloud<pcl::PointXYZI>& cloud) const  // //{
{
  auto             cloud_with_seeds = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(cloud);
  std::vector<int> seed_indices;

  for (const auto& track : tracks_) {
    pcl::PointXYZI seed_point;
    seed_point.x         = static_cast<float>(track.x(0));
    seed_point.y         = static_cast<float>(track.x(1));
    seed_point.z         = static_cast<float>(track.x(2));
    seed_point.intensity = static_cast<float>(params_.max_intensity);
    seed_indices.push_back(static_cast<int>(cloud_with_seeds->points.size()));
    cloud_with_seeds->points.push_back(seed_point);
  }

  cloud_with_seeds->width    = static_cast<std::uint32_t>(cloud_with_seeds->points.size());
  cloud_with_seeds->height   = 1;
  cloud_with_seeds->is_dense = false;

  if (cloud_with_seeds->points.empty() || seed_indices.empty()) {
    auto passthrough_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(cloud);
    passthrough_cloud->width    = static_cast<std::uint32_t>(passthrough_cloud->points.size());
    passthrough_cloud->height   = 1;
    passthrough_cloud->is_dense = false;
    return passthrough_cloud;
  }

  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(cloud_with_seeds);

  std::vector<bool> is_uav_point(cloud_with_seeds->points.size(), false);
  const float       max_sq_distance_from_seed =
      static_cast<float>(params_.max_distance_from_seed * params_.max_distance_from_seed);

  struct QueueElement {
    int   idx;
    float sq_dist_from_seed;
  };

  for (const int seed_index : seed_indices) {
    std::queue<QueueElement> queue;
    queue.push({seed_index, 0.0f});
    const auto& seed = cloud_with_seeds->points[static_cast<std::size_t>(seed_index)];
    is_uav_point[static_cast<std::size_t>(seed_index)] = true;

    while (!queue.empty()) {
      const auto current_element = queue.front();
      queue.pop();

      std::vector<int>   neighbors;
      std::vector<float> sqr_distances_to_neighbor;
      (void)current_element.sq_dist_from_seed;

      kdtree.radiusSearch(
          cloud_with_seeds->points[static_cast<std::size_t>(current_element.idx)],
          static_cast<float>(params_.search_radius),
          neighbors,
          sqr_distances_to_neighbor);

      for (const int neighbor_idx : neighbors) {
        const auto& neighbor_point = cloud_with_seeds->points[static_cast<std::size_t>(neighbor_idx)];
        const float dx             = neighbor_point.x - seed.x;
        const float dy             = neighbor_point.y - seed.y;
        const float dz             = neighbor_point.z - seed.z;
        const float sq_dist_to_seed = dx * dx + dy * dy + dz * dz;

        if (sq_dist_to_seed > max_sq_distance_from_seed ||
            is_uav_point[static_cast<std::size_t>(neighbor_idx)]) {
          continue;
        }

        is_uav_point[static_cast<std::size_t>(neighbor_idx)] = true;
        queue.push({neighbor_idx, sq_dist_to_seed});
      }
    }
  }

  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  filtered_cloud->points.reserve(cloud.points.size());

  int removed_points = 0;
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    if (is_uav_point[i]) {
      ++removed_points;
      if (params_.max_removed_points > 0 && removed_points > params_.max_removed_points) {
        filtered_cloud->points.push_back(cloud.points[i]);
      }
      continue;
    }

    filtered_cloud->points.push_back(cloud.points[i]);
  }

  filtered_cloud->width    = static_cast<std::uint32_t>(filtered_cloud->points.size());
  filtered_cloud->height   = 1;
  filtered_cloud->is_dense = false;
  return filtered_cloud;
}  // //}

}  // namespace filter_reflective_uavs
