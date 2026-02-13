#ifndef CIRI_H
#define CIRI_H

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include <iostream>


class Ellipsoid {
  /// If the ellipsoid is empty
  bool undefined{true};

  /// The ellipsoid is defined by shape C and center d
  Eigen::Matrix3f C_{}, C_inv_{};
  Eigen::Matrix3f R_{};
  Eigen::Vector3f r_{}, d_{};

public:
  Ellipsoid() = default;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(undefined, C_, C_inv_, R_, r_, d_);
  }

  Ellipsoid(const Eigen::Matrix3f& C, const Eigen::Vector3f& d);

  Ellipsoid(const Eigen::Matrix3f& R, const Eigen::Vector3f& r, const Eigen::Vector3f& d);

  /// If this ellipsoid is empty
  bool empty() const;

  double pointDistaceToEllipsoid(const Eigen::Vector3f& pt, Eigen::Vector3f& closest_pt_on_ellip) const;

  double DistancePointEllipsoid(double e0, double e1, double e2, double y0, double y1, double y2, double& x0, double& x1, double& x2) const;

  double DistancePointEllipse(double e0, double e1, double y0, double y1, double& x0, double& x1) const;

  /// Find the closestPoint in a point set
  int nearestPointId(const Eigen::Matrix3Xf& pc) const;

  /// Find the closestPoint in a point set
  Eigen::Vector3f nearestPoint(const Eigen::Matrix3Xf& pc) const;

  /// Find the closestPoint in a point set
  double nearestPointDis(const Eigen::Matrix3Xf& pc, int& np_id) const;

  /// Get the shape of the ellipsoid
  Eigen::Matrix3f C() const;

  /// Get the center of the ellipsoid
  Eigen::Vector3f d() const;

  Eigen::Matrix3f R() const;

  Eigen::Vector3f r() const;

  /// Convert a point to the ellipsoid frame
  Eigen::Vector3f toEllipsoidFrame(const Eigen::Vector3f& pt_w) const;

  // Eigen::Vector3d toEllipsoidFrame(const Eigen::Vector3d &pt_w) const;

  /// Convert a set of points to the ellipsoid frame
  // Eigen::Matrix3Xd toEllipsoidFrame(const Eigen::Matrix3Xd& pc_w) const;

  Eigen::Matrix3Xf toEllipsoidFrame(const Eigen::Matrix3Xf& pc_w) const;

  /// Convert a point to the world frame
  Eigen::Vector3f toWorldFrame(const Eigen::Vector3f& pt_e) const;

  /// Convert a set of points to the world frame
  Eigen::Matrix3Xf toWorldFrame(const Eigen::Matrix3Xf& pc_e) const;

  /// Convert a plane to the ellipsoid frame
  // Eigen::Vector4d toEllipsoidFrame(const Eigen::Vector4d& plane_w) const;

  /// Convert a plane to the ellipsoid frame
  // Eigen::Vector4d toWorldFrame(const Eigen::Vector4d& plane_e) const;

  Eigen::Vector4f toWorldFrame(const Eigen::Vector4f& plane_e) const;

  /// Convert a set of planes to ellipsoid frame
  // Eigen::MatrixX4d toEllipsoidFrame(const Eigen::MatrixX4d& planes_w) const;

  Eigen::MatrixX4f toEllipsoidFrame(const Eigen::MatrixX4f& planes_w) const;

  /// Convert a set of planes to ellipsoid frame
  // Eigen::MatrixX4d toWorldFrame(const Eigen::MatrixX4d& planes_e) const;

  /// Calculate the distance of a point in world frame
  double dist(const Eigen::Vector3f& pt_w) const;

  /// Calculate the distance of a point in world frame
  // Eigen::VectorXd dist(const Eigen::Matrix3Xd& pc_w) const;

  Eigen::VectorXf dist(const Eigen::Matrix3Xf& pc_w) const;

  // bool noPointsInside(std::vector<Eigen::Vector3f>& pc, const Eigen::Matrix3d& R, const Eigen::Vector3f& r, const Eigen::Vector3f& p) const;

  bool pointsInside(const Eigen::Matrix3Xf& pc, Eigen::Matrix<float, 3, 3>& out, int& min_pt_id) const;

  // bool pointsInside(const Eigen::Matrix3Xd& pc, Eigen::Matrix<double, 3, 3>& out, int& min_pt_id) const;

  bool pointsInside(const Eigen::Matrix<float, 3, 3>& pc, Eigen::Matrix<float, 3, 3>& out, int& min_pt_id) const;

  /// Check if the point is inside, non-exclusive
  bool inside(const Eigen::Vector3f& pt) const;

};


struct ciriParams {
  double                                inflation;
  double                                epsilon                       = 1e-6;
};


class CIRI {
public:
  CIRI(const ciriParams& params);
  bool comvexDecomposition(const Eigen::MatrixX4f& bd, const Eigen::Matrix3Xf& pc, const Eigen::Vector3f& a, const Eigen::Vector3f& b);
  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> getPlaneData(); 

private:
  ciriParams                                                    params_;
  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>      plane_data_;
  int                                                           iter_num_{1};
  Ellipsoid                                                     sphere_template_;

  void findTangentPlaneOfSphere(const Eigen::Vector3f& center, const double& r, const Eigen::Vector3f& pass_point, const Eigen::Vector3f& seed_p, Eigen::Vector4f& outter_plane);
  void findEllipsoid(const Eigen::Matrix3Xf &pc, const Eigen::Vector3f &a, const Eigen::Vector3f &b, Ellipsoid &out_ell);
  void convertToPlaneData(const Eigen::Matrix<float, Eigen::Dynamic, 4>& hPoly, std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& plane_data);
  double distancePointToSegment(const Eigen::Vector3f& P, const Eigen::Vector3f& A, const Eigen::Vector3f& B);
};


#endif