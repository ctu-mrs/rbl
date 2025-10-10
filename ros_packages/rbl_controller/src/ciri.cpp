#include "ciri.h"

CIRI::CIRI(const ciriParams& params) : params_(params)// //{
{
  sphere_template_ = Ellipsoid(Eigen::Matrix3f::Identity(), params.inflation * Eigen::Vector3f(1, 1, 1), Eigen::Vector3f(0, 0, 0));
}// //}

bool CIRI::comvexDecomposition(const Eigen::MatrixX4f& bd, const Eigen::Matrix3Xf& pc, const Eigen::Vector3f& a, const Eigen::Vector3f& b) // //{
// bd -> boundary, each row (n_x, n_y, n_z, d) which forms constraint n_xx + n_yy + n_zz + d <= 0. Should be boundary of the map? 
// pc -> point cloud of obstacles
// a -> seed for ellipsoid (agent pos)
// b -> sedd for ellipsoid (waypoint)
{
  const Eigen::Vector4f ah(a(0), a(1), a(2), 1.0);
  const Eigen::Vector4f bh(b(0), b(1), b(2), 1.0);

  if ((bd * ah).maxCoeff() > params_.epsilon || (bd * bh).maxCoeff() > params_.epsilon) {
    std::cout << "[CIRI]: Seeds points for ellipsoid lies outside the boundary region. Unable to construct polyhedron." << std::endl;
    return false;
  }

  /// Maximum M boundary constraints and N point constraints
  const int M = bd.rows();
  const int N = pc.cols();

  Ellipsoid E(Eigen::Matrix3f::Identity(), (a + b) / 2);

  if ((a - b).norm() > 0.1) {
    /// use line seed
    findEllipsoid(pc, a, b, E);
  }

  std::vector<Eigen::Vector4f> planes;
  Eigen::Matrix<float, Eigen::Dynamic, 4> hPoly;

  Eigen::Vector3f infeasible_pt_w;

  for (int loop = 0; loop < iter_num_; ++loop) {
    // Initialize the boundary in ellipsoid frame
    const Eigen::Vector3f fwd_a = E.toEllipsoidFrame(a);
    const Eigen::Vector3f fwd_b = E.toEllipsoidFrame(b);
    const Eigen::MatrixX4f bd_e = E.toEllipsoidFrame(bd);
    const Eigen::VectorXf distDs = bd_e.rightCols<1>().cwiseAbs().cwiseQuotient(bd_e.leftCols<3>().rowwise().norm());
    const Eigen::Matrix3Xf pc_e = E.toEllipsoidFrame(pc);
    Eigen::VectorXf distRs = pc_e.colwise().norm();

    Eigen::Matrix<uint8_t, -1, 1> bdFlags = Eigen::Matrix<uint8_t, -1, 1>::Constant(M, 1);
    Eigen::Matrix<uint8_t, -1, 1> pcFlags = Eigen::Matrix<uint8_t, -1, 1>::Constant(N, 1);

    bool completed = false;
    int bdMinId, pcMinId;
    double minSqrD = distDs.minCoeff(&bdMinId);
    double minSqrR = distRs.minCoeff(&pcMinId);

    Eigen::Vector4f temp_tangent, temp_plane_w;
    const Eigen::Matrix3f C_inv = E.C().inverse();

    planes.clear();
    planes.reserve(30);
    Eigen::Vector3f tmp_nn_pt;
    Eigen::Vector4f plan_before_ab;

    for (int i = 0; !completed && i < (M + N); ++i) {
      if (minSqrD < minSqrR) {
        /// Case [Bd closer than ob]  enable the boundary constrain.
        Eigen::Vector4f p_e = bd_e.row(bdMinId);
        temp_plane_w = E.toWorldFrame(p_e);
        bdFlags(bdMinId) = 0;
      }
      else {
        /// Case [Ob closer than Bd] enable the obstacle point constarin.
        ///     Compute the tangent plane of sphere
        ///
        const auto & pt_w = pc.col(pcMinId);
        const auto dis = distancePointToSegment(pt_w,a,b);
        if(dis < params_.inflation - 1e-2) {
          infeasible_pt_w = pt_w;
          std::cout << "[CIRI]: WARNING! The problem is not feasible, the min dis to obstacle is only: "<< dis << std::endl;
          return false;
          // cout<<" -- [CIRI] dis: "<<dis<<endl;
          // cout<<" -- [CIRI] robot_r: "<<params_.robot_r<<endl;
          // cout<<" -- [CIRI] pcMin: "<<pt_w.transpose()<<endl;
          // cout<<" -- [CIRI] a: "<<a.transpose()<<endl;
          // cout<<" -- [CIRI] b: "<<b.transpose()<<endl;
        }

        if (params_.inflation < params_.epsilon) {
          const Eigen::Vector3f& pt_e = pc_e.col(pcMinId);
          temp_tangent(3) = -distRs(pcMinId);
          temp_tangent.head(3) = pt_e.transpose() / distRs(pcMinId);

          if (temp_tangent.head(3).dot(fwd_a) + temp_tangent(3) > params_.epsilon) {
            const Eigen::Vector3f delta = pc_e.col(pcMinId) - fwd_a;
            temp_tangent.head(3) = fwd_a - (delta.dot(fwd_a) / delta.squaredNorm()) * delta;
            distRs(pcMinId) = temp_tangent.head(3).norm();
            temp_tangent(3) = -distRs(pcMinId);
            temp_tangent.head(3) /= distRs(pcMinId);
          }
          if (temp_tangent.head(3).dot(fwd_b) + temp_tangent(3) > params_.epsilon) {
            const Eigen::Vector3f delta = pc_e.col(pcMinId) - fwd_b;
            temp_tangent.head(3) = fwd_b - (delta.dot(fwd_b) / delta.squaredNorm()) * delta;
            distRs(pcMinId) = temp_tangent.head(3).norm();
            temp_tangent(3) = -distRs(pcMinId);
            temp_tangent.head(3) /= distRs(pcMinId);
          }
          if (temp_tangent.head(3).dot(fwd_b) + temp_tangent(3) > params_.epsilon) {
            const Eigen::Vector3f delta = pc_e.col(pcMinId) - fwd_b;
            temp_tangent.head(3) = fwd_b - (delta.dot(fwd_b) / delta.squaredNorm()) * delta;
            distRs(pcMinId) = temp_tangent.head(3).norm();
            temp_tangent(3) = -distRs(pcMinId);
            temp_tangent.head(3) /= distRs(pcMinId);
          }
          temp_plane_w = E.toWorldFrame(temp_tangent);
        }
        else {
          /// Case [Ob closer than Bd] enable the obstacle point constarin.
          const Eigen::Vector3f &pt_e = pc_e.col(pcMinId);
          const Eigen::Vector3f &pt_w = pc.col(pcMinId);
          Ellipsoid E_pe(C_inv * sphere_template_.C(), pt_e);
          Eigen::Vector3f close_pt_e;
          E_pe.pointDistaceToEllipsoid(Eigen::Vector3f(0, 0, 0), close_pt_e);
          Eigen::Vector3f c_pt_w = E.toWorldFrame(close_pt_e);
          temp_plane_w.head(3) = (pt_w - c_pt_w).normalized();
          temp_plane_w(3) = -temp_plane_w.head(3).dot(c_pt_w);

          /// Cut line with sphere A and B,
          if (temp_plane_w.head(3).dot(a) + temp_plane_w(3) > -params_.epsilon) {
              // Case the plan make seed out, the plane should be modified in world frame
              findTangentPlaneOfSphere(pt_w, params_.inflation, a, E.d(), temp_plane_w);
          } else if (temp_plane_w.head(3).dot(b) + temp_plane_w(3) > -params_.epsilon) {
              // Case the plan make seed out, the plane should be modified in world frame
              findTangentPlaneOfSphere(pt_w, params_.inflation, b, E.d(), temp_plane_w);
          }
        }
        pcFlags(pcMinId) = 0;
        tmp_nn_pt = pc.col(pcMinId);
      }
      // update pcMinId and bdMinId
      completed = true;
      minSqrD = INFINITY;
      for (int j = 0; j < M; ++j) {
        if (bdFlags(j)) {
          completed = false;
          if (minSqrD > distDs(j)) {
            bdMinId = j;
            minSqrD = distDs(j);
          }
        }
      }
      minSqrR = INFINITY;
      for (int j = 0; j < N; ++j) {
        if (pcFlags(j)) {
          if ((temp_plane_w.head(3).dot(pc.col(j)) + temp_plane_w(3)) > params_.inflation - params_.epsilon) {
            pcFlags(j) = 0;
          }
          else {
            completed = false;
            if (minSqrR > distRs(j)) {
              pcMinId = j;
              minSqrR = distRs(j);
            }
          }
        }
      }
      planes.push_back(temp_plane_w);
      if (planes.size() > 50){
        std::cout << "[CIRI]: WARNING! plane count exceeded limit" << std::endl;
        return false;
      }
    }    

    hPoly.resize(planes.size(), 4);
    for (size_t i = 0; i < planes.size(); ++i) {
      hPoly.row(i) = planes[i];
    }

    if (loop == iter_num_ - 1) {
      break;
    }

    if(hPoly.array().isNaN().any()) {
      std::cout << "[CIRI]: ERROR! maxVolInsEllipsoid failed." << std::endl;
      return false;
    }

    // if (!MVIE::maxVolInsEllipsoid(hPoly, E)) {
    //   return false;
    // }
  }
  // if (std::isnan(hPoly.sum())) {
  //   cout << YELLOW << " -- [CIRI] ERROR! There is nan in generated planes." << RESET << endl;
  //   cout << a.transpose() << endl;
  //   cout << b.transpose() << endl;
  //   ros_ptr_->vizCiriSeedLine(a, b,params_.robot_r);
  //   ros_ptr_->vizCiriEllipsoid(E);
  //   return FAILED;
  // }
  // Eigen::Vector3f inner;
  // if (!geometry_utils::findInterior(hPoly, inner)) {
  //   cout<<RED<<" -- [CIRI] The polytope is empty."<<RESET<<endl;
  //   ros_ptr_->vizCiriSeedLine(a, b,params_.robot_r);
  //   ros_ptr_->vizCiriEllipsoid(E);
  //   return FAILED;
  // }
  // optimized_polytope_.Reset();
  // optimized_polytope_.SetPlanes(hPoly);
  // optimized_polytope_.SetSeedLine(std::make_pair(a, b));
  // optimized_polytope_.SetEllipsoid(E);
  convertToPlaneData(hPoly, plane_data_);
  return true;  
}// //}

void CIRI::findTangentPlaneOfSphere(const Eigen::Vector3f& center, const double& r,// //{
                                    const Eigen::Vector3f& pass_point,
                                    const Eigen::Vector3f& seed_p,
                                    Eigen::Vector4f& outter_plane) 
{
  Eigen::Vector3f seed = seed_p;
  Eigen::Vector3f dif = pass_point - pass_point;
  if (dif.norm() < 1e-3) {
    if ((pass_point - center).head(2).norm() > 1e-3) {
      Eigen::Vector3f v1 = (pass_point - center).normalized();
      v1(2) = 0;
      seed = seed_p + 0.01 * v1.cross(Eigen::Vector3f(0, 0, 1)).normalized();
    }
    else {
      seed = seed_p + 0.01 * (pass_point - center).cross(Eigen::Vector3f(1, 0, 0)).normalized();
    }
  }
  Eigen::Vector3f P = pass_point - center;
  Eigen::Vector3f norm_ = (pass_point - center).cross(seed - center).normalized();
  Eigen::Matrix3f R = Eigen::Quaternionf::FromTwoVectors(norm_, Eigen::Vector3f(0, 0, 1)).matrix();
  P = R * P;
  Eigen::Vector3f C = R * (seed - center);
  Eigen::Vector3f Q;
  double r2 = r * r;
  double p1p2n = P.head(2).squaredNorm();
  double d = sqrt(p1p2n - r2);
  double rp1p2n = r / p1p2n;
  double q11 = rp1p2n * (P(0) * r - P(1) * d);
  double q21 = rp1p2n * (P(1) * r + P(0) * d);

  double q12 = rp1p2n * (P(0) * r + P(1) * d);
  double q22 = rp1p2n * (P(1) * r - P(0) * d);
  if (q11 * C(0) + q21 * C(1) < 0) {
    Q(0) = q12;
    Q(1) = q22;
  }
  else {
    Q(0) = q11;
    Q(1) = q21;
  }
  Q(2) = 0;
  outter_plane.head(3) = R.transpose() * Q;
  Q = outter_plane.head(3) + center;
  outter_plane(3) = -Q.dot(outter_plane.head(3));
  if (outter_plane.head(3).dot(seed) + outter_plane(3) > params_.epsilon) {
    outter_plane = -outter_plane;
  }
}// //}

double CIRI::distancePointToSegment(const Eigen::Vector3f& P, const Eigen::Vector3f& A, const Eigen::Vector3f& B) // //{
{
  Eigen::Vector3f AB = B - A;
  Eigen::Vector3f AP = P - A;

  double AB_AB = AB.dot(AB);  // AB·AB
  double AP_AB = AP.dot(AB);  // AP·AB
  double t = AP_AB / AB_AB;

  if (t < 0.0f) {
    return (P - A).norm();
  } else if (t > 1.0f) {
    return (P - B).norm();
  } else {
    Eigen::Vector3f Q = A + t * AB;
    return (P - Q).norm();
  }
}// //}

void CIRI::findEllipsoid(const Eigen::Matrix3Xf& pc, const Eigen::Vector3f& a, const Eigen::Vector3f& b, Ellipsoid& out_ell) // //{
{
  double f = (a - b).norm() / 2;
  Eigen::Matrix3f C = f * Eigen::Matrix3f::Identity();
  Eigen::Vector3f r = Eigen::Vector3f::Constant(f);
  Eigen::Vector3f center = (a + b) / 2;
  C(0, 0) += params_.inflation;
  r(0) += params_.inflation;
  if (r(0) > 0) {
    double ratio = r(1) / r(0);
    r *= ratio;
    C *= ratio;
  }

  Eigen::Matrix3f Ri = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitX(), (b - a)).toRotationMatrix();
  Ellipsoid E(Ri, r, center);
  Eigen::Matrix3f Rf = Ri;
  Eigen::Matrix<float, 3, 3> obs;
  int min_dis_id;
  Eigen::Vector3f pw;
  if (E.pointsInside(pc, obs, min_dis_id)) {
    pw = obs.col(min_dis_id).cast<float>();;
  }
  else {
    out_ell = E;
    return;
  }
  Eigen::Matrix<float, 3, 3> obs_inside = obs;
  int max_iter = 100;
  while (max_iter--) {
    Eigen::Vector3f p_e = Ri.transpose() * (pw - E.d());
    const double roll = atan2(p_e(2), p_e(1));
    Rf = Ri * Eigen::Quaternionf(cos(roll / 2), sin(roll / 2), 0, 0);
    p_e = Rf.transpose() * (pw - E.d());
    if (p_e(0) < r(0)) {
      r(1) = std::abs(p_e(1)) / std::sqrt(1 - std::pow(p_e(0) / r(0), 2));
    }
    E = Ellipsoid(Rf, r, center);
    if (E.pointsInside(obs_inside, obs_inside, min_dis_id)) {
      pw = obs_inside.col(min_dis_id).cast<float>();;
    }
    else {
      break;
    }
  }
  if (max_iter == 0) {
    std::cout << "[CIRI]: Find Ellipsoid reach max iteration, may cause error." << std::endl;
  }
  max_iter = 100;

  if (E.pointsInside(obs, obs_inside, min_dis_id)) {
    pw = obs_inside.col(min_dis_id).cast<float>();;
  }
  else {
    out_ell = E;
    return;
  }

  while (max_iter--) {
    Eigen::Vector3f p = Rf.transpose() * (pw - E.d());
    double dd = 1 - std::pow(p(0) / r(0), 2) - std::pow(p(1) / r(1), 2);
    if (dd > params_.epsilon) {
      r(2) = std::abs(p(2)) / std::sqrt(dd);
    }
    E = Ellipsoid(Rf, r, center);
    if (E.pointsInside(obs_inside, obs_inside, min_dis_id)) {
      pw = obs_inside.col(min_dis_id).cast<float>();;
    }
    else {
      out_ell = E;
      break;
    }
  }

  if (max_iter == 0) {
    std::cout << "[CIRI]: Find Ellipsoid reach max iteration, may cause error." << std::endl;
  }
  E = Ellipsoid(Rf, r, center);
  out_ell = E;
}// //}

void CIRI::convertToPlaneData(const Eigen::Matrix<float, Eigen::Dynamic, 4>& hPoly, std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& plane_data) // //{
{
  plane_data.clear();
  plane_data.reserve(hPoly.rows());

  for (int i = 0; i < hPoly.rows(); ++i) {
    Eigen::Vector3f normal = hPoly.row(i).head(3);
    float d = hPoly(i, 3);
    
    float normal_squared_norm = normal.squaredNorm();
    if (normal_squared_norm < 1e-6) { // Avoid division by zero
      continue;
    }
    
    normal.normalize();
    
    Eigen::Vector3f point_on_plane = normal * (-d);
    
    plane_data.push_back({normal, point_on_plane});
  }
}// //}

std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> CIRI::getPlaneData() // //{
{
  return plane_data_;
}// //}

//____________________________________________________________________________________________________________Ellipsoid____________________________________________________________________________________________________________
//

bool Ellipsoid::empty() const// //{
{
  return undefined;
}// //}

Ellipsoid::Ellipsoid(const Eigen::Matrix3f &C, const Eigen::Vector3f &d) : C_(C), d_(d) // //{
{
  undefined = false;
  C_inv_ = C_.inverse();

  Eigen::JacobiSVD<Eigen::Matrix3f, Eigen::FullPivHouseholderQRPreconditioner> svd(C_, Eigen::ComputeFullU);
  Eigen::Matrix3f U = svd.matrixU();
  Eigen::Vector3f S = svd.singularValues();
  if (U.determinant() < 0.0) {
    R_.col(0) = U.col(1);
    R_.col(1) = U.col(0);
    R_.col(2) = U.col(2);
    r_(0) = S(1);
    r_(1) = S(0);
    r_(2) = S(2);
  } else {
    R_ = U;
    r_ = S;
  }
}// //}

Ellipsoid::Ellipsoid(const Eigen::Matrix3f &R, const Eigen::Vector3f &r, const Eigen::Vector3f &d) : R_(R), r_(r), d_(d) // //{
{
  undefined = false;
  C_ = R_ * r_.asDiagonal() * R_.transpose();
  C_inv_ = C_.inverse();
}// //}

double Ellipsoid::pointDistaceToEllipsoid(const Eigen::Vector3f &pt, Eigen::Vector3f &closest_pt_on_ellip) const // //{
{
  /// step one: transform the point to the ellipsoid frame
  Eigen::Vector3f pt_ellip_frame = R_.transpose() * (pt - d_);
  double closest_x, closest_y, closest_z;
  double dist = DistancePointEllipsoid(r_(0), r_(1), r_(2),
                                                        pt_ellip_frame.x(),
                                                        pt_ellip_frame.y(),
                                                        pt_ellip_frame.z(),
                                                        closest_x,
                                                        closest_y,
                                                        closest_z);
  closest_pt_on_ellip = Eigen::Vector3f(closest_x, closest_y, closest_z);
  /// step two: transform the closest point back to the world frame
  closest_pt_on_ellip = R_ * closest_pt_on_ellip + d_;
  return dist;
}// //}

double Ellipsoid::DistancePointEllipsoid(double e0, double e1, double e2,// //{
                       double y0, double y1, double y2,
                       double& x0, double& x1, double& x2) const 
{
  auto getRoot = [&](double r0, double r1, double z0, double z1, double z2, double g) {
  double n0 = r0 * z0, n1 = r1 * z1;
  double s0 = z2 - 1, s1 = (g < 0 ? 0 : sqrt(n0 * n0 + n1 * n1 + z2 * z2) - 1);
  double s = 0;
  const int maxIterations = 10;
  for (int i = 0; i < maxIterations; ++i) {
    s = (s0 + s1) / 2;
    if (s == s0 || s == s1) {
      break;
    }
    double ratio0 = n0 / (s + r0), ratio1 = n1 / (s + r1), ratio2 = z2 / (s + 1);
    g = (ratio0 * ratio0) + (ratio1 * ratio1) + (ratio2 * ratio2) - 1;
    if (g > 0) {
      s0 = s;
    }
    else if (g < 0) {
      s1 = s;
    }
    else {
      break;
    }
  }
  return s;
  };
  constexpr double eps = 1e-8;
  double distance;
  double record_sign[3] = {1, 1, 1};

  if (y0 < 0) {
    record_sign[0] = -1;
    y0 = -y0;
  }
  if (y1 < 0) {
    record_sign[1] = -1;
    y1 = -y1;
  }
  if (y2 < 0) {
    record_sign[2] = -1;
    y2 = -y2;
  }

  if (y2 > eps) {
    if (y1 > eps) {
      if (y0 > eps) {
        double z0 = y0 / e0, z1 = y1 / e1, z2 = y2 / e2;
        double g = sqrt(z0 * z0 + z1 * z1 + z2 * z2) - 1;

        if (g != 0) {
          double r0 = e0 * e0 / e2 / e2, r1 = e1 * e1 / e2 / e2;
          double sbar = getRoot(r0, r1, z0, z1, z2, g);

          x0 = r0 * y0 / (sbar + r0);
          x1 = r1 * y1 / (sbar + r1);
          x2 = y2 / (sbar + 1);

          distance = sqrt((x0 - y0) * (x0 - y0) +
                (x1 - y1) * (x1 - y1) +
                (x2 - y2) * (x2 - y2));
        }
        else {
          x0 = y0;
          x1 = y1;
          x2 = y2;
          distance = 0;
        }
      }
      else // y0 == 0
      {
        x0 = 0;
        distance = DistancePointEllipse(e1, e2, y1, y2, x1, x2);
      }
    }
    else // y1 == 0
    {
      if (y0 > 0) {
        x1 = 0;
        distance = DistancePointEllipse(e0, e2, y0, y2, x0, x2);
      }
      else // y0 == 0
      {
        x0 = 0;
        x1 = 0;
        x2 = e2;
        distance = fabs(y2 - e2);
      }
    }
  }
  else // y2 == 0
  {
    double denom0 = e0 * e0 - e2 * e2, denom1 = e1 * e1 - e2 * e2;
    double numer0 = e0 * y0, numer1 = e1 * y1;
    bool computed = false;

    if (numer0 < denom0 && numer1 < denom1) {
      double xde0 = numer0 / denom0, xde1 = numer1 / denom1;
      double discr = 1 - xde0 * xde0 - xde1 * xde1;

      if (discr > 0) {
        x0 = e0 * xde0;
        x1 = e1 * xde1;
        x2 = e2 * sqrt(discr);

        distance = sqrt((x0 - y0) * (x0 - y0) +
            (x1 - y1) * (x1 - y1) +
            x2 * x2);
        computed = true;
      }
    }
    if (!computed) {
      x2 = 0;
      distance = DistancePointEllipse(e0, e1, y0, y1, x0, x1);
    }
  }
  x0 *= record_sign[0];
  x1 *= record_sign[1];
  x2 *= record_sign[2];
  return distance;
}// //}

double Ellipsoid::DistancePointEllipse(double e0, double e1,// //{
                            double y0, double y1,
                            double& x0, double& x1) const 
{
  double distance;
  double record_sign[2] = {1, 1};
  constexpr double eps = 1e-8;
  if (y0 < 0) {
    record_sign[0] = -1;
    y0 = -y0;
  }
  if (y1 < 0) {
    record_sign[1] = -1;
    y1 = -y1;
  }

  auto getRoot = [&](double r0, double z0, double z1, double g) {
    double n0 = r0 * z0;
    double s0 = z1 - 1, s1 = (g < 0 ? 0 : sqrt(n0 * n0 + z1 * z1) - 1);
    double s = 0;
    for (int i = 0; i < 10; ++i) {
      s = (s0 + s1) / 2;
      if (s == s0 || s == s1) {
        break;
      }
      double ratio0 = n0 / (s + r0), ratio1 = z1 / (s + 1);
      g = ratio0 * ratio0 + ratio1 * ratio1 - 1;
      if (g > 0) {
        s0 = s;
      }
      else if (g < 0) {
        s1 = s;
      }
      else {
        break;
      }
    }
    return s;
  };

  if (y1 > eps) {
    if (y0 > eps) {
      double z0 = y0 / e0, z1 = y1 / e1, g = z0 * z0 + z1 * z1 - 1;
      if (g != 0) {
        double r0 = e0 * e0 / e1 / e1, sbar = getRoot(r0, z0, z1, g);
        x0 = r0 * y0 / (sbar + r0);
        x1 = y1 / (sbar + 1);
        distance = sqrt((x0 - y0) * (x0 - y0) + (x1 - y1) * (x1 - y1));
      }
      else {
        x0 = y0;
        x1 = y1;
        distance = 0;
      }
    }
    else {
      // y0 == 0
      x0 = 0;
      x1 = e1;
      distance = fabs(y1 - e1);
    }
  }
  else {
    // y1 == 0
    double numer0 = e0 * y0, denom0 = e0 * e0 - e1 * e1;
    if (numer0 < denom0) {
      double xde0 = numer0 / denom0;
      x0 = e0 * xde0;
      x1 = e1 * sqrt(1 - xde0 * xde0);
      distance = sqrt((x0 - y0) * (x0 - y0) + x1 * x1);
    }
    else {
      x0 = e0;  
      x1 = 0;
      distance = fabs(y0 - e0);
    }
  }
  x0 *= record_sign[0];
  x1 *= record_sign[1];

  return distance;
}// //}

int Ellipsoid::nearestPointId(const Eigen::Matrix3Xf &pc) const // //{
{
  Eigen::VectorXf dists = (C_inv_ * (pc.colwise() - d_)).colwise().norm();
  // Eigen::VectorXd dists = (C_inv_ * (pc.colwise() - d_)).colwise().norm();
  int np_id;
  dists.minCoeff(&np_id);
  return np_id;
}// //}

Eigen::Vector3f Ellipsoid::nearestPoint(const Eigen::Matrix3Xf &pc) const // //{
{
  Eigen::VectorXf dists = (C_inv_ * (pc.colwise() - d_)).colwise().norm();
  int np_id;
  dists.minCoeff(&np_id);
  return pc.col(np_id);
}// //}

double Ellipsoid::nearestPointDis(const Eigen::Matrix3Xf &pc, int &np_id) const // //{
{
  Eigen::VectorXf dists = (C_inv_ * (pc.colwise() - d_)).colwise().norm();
  double np_dist = dists.minCoeff(&np_id);
  return np_dist;
}// //}

Eigen::Matrix3f Ellipsoid::C() const {// //{
  return C_;
}// //}

Eigen::Vector3f Ellipsoid::d() const {// //{
  return d_;
}// //}

Eigen::Matrix3f Ellipsoid::R() const {// //{
  return R_;
}// //}

Eigen::Vector3f Ellipsoid::r() const {// //{
  return r_;
}// //}

Eigen::Vector3f Ellipsoid::toEllipsoidFrame(const Eigen::Vector3f &pt_w) const {// //{
  return C_inv_ * (pt_w - d_);
}// //}

Eigen::Matrix3Xf Ellipsoid::toEllipsoidFrame(const Eigen::Matrix3Xf &pc_w) const {// //{
  return C_inv_ * (pc_w.colwise() - d_);
}// //}

Eigen::Vector3f Ellipsoid::toWorldFrame(const Eigen::Vector3f &pt_e) const {// //{
  return C_ * pt_e + d_;
}// //}

Eigen::Matrix3Xf Ellipsoid::toWorldFrame(const Eigen::Matrix3Xf &pc_e) const {// //{
  return (C_ * pc_e).colwise() + d_;
}// //}

Eigen::Vector4f Ellipsoid::toWorldFrame(const Eigen::Vector4f &plane_e) const {// //{
  Eigen::Vector4f plane_w;
  plane_w.head(3) = plane_e.head(3).transpose() * C_inv_;
  plane_w(3) = plane_e(3) - plane_w.head(3).dot(d_);
  return plane_w;
}// //}

Eigen::MatrixX4f Ellipsoid::toEllipsoidFrame(const Eigen::MatrixX4f &planes_w) const {// //{
  Eigen::MatrixX4f planes_e(planes_w.rows(), planes_w.cols());
  planes_e.leftCols(3) = planes_w.leftCols(3) * C_;
  planes_e.rightCols(1) = planes_w.rightCols(1) + planes_w.leftCols(3) * d_;
  return planes_e;
}// //}

double Ellipsoid::dist(const Eigen::Vector3f &pt_w) const {// //{
  return (C_inv_ * (pt_w - d_)).norm();
}// //}

Eigen::VectorXf Ellipsoid::dist(const Eigen::Matrix3Xf &pc_w) const {// //{
  return (C_inv_ * (pc_w.colwise() - d_)).colwise().norm();
}// //}

bool Ellipsoid::pointsInside(const Eigen::Matrix3Xf &pc, Eigen::Matrix<float, 3, 3> &out, int &min_pt_id) const // //{
{
  Eigen::VectorXf vec = (C_inv_ * (pc.colwise() - d_)).colwise().norm();
  std::vector<Eigen::Vector3f> pts;
  pts.reserve(pc.cols());
  int cnt = 0;
  min_pt_id = 0;
  float min_dis = std::numeric_limits<float>::max();
  for (long int i = 0; i < vec.size(); i++) {
    if (vec(i) <= 1) {
      pts.push_back(pc.col(i));
      if (vec(i) <= min_dis) {
        min_pt_id = cnt;
        min_dis = vec(i);
      }
      cnt++;
    }
  }
  if (!pts.empty()) {
    // out.resize(3, pts.size());
    // for (size_t i = 0; i < pts.size(); i++) {
    //   out.col(i) = pts[i];
    // }
    out = Eigen::Map<const Eigen::Matrix<float, 3, -1, Eigen::ColMajor>>(pts[0].data(), 3, pts.size());
    return true;
  } else {
    return false;
  }
}// //}

bool Ellipsoid::pointsInside(const Eigen::Matrix<float, 3, 3>& pc, Eigen::Matrix<float, 3, 3>& out, int& min_pt_id) const // //{
{
  Eigen::VectorXf vec = (C_inv_ * (pc.colwise() - d_)).colwise().norm();
  std::vector<Eigen::Vector3f> pts;
  pts.reserve(pc.cols());
  int cnt = 0;
  min_pt_id = 0;
  double min_dis = std::numeric_limits<double>::max();
  for (long int i = 0; i < vec.size(); i++) {
    if (vec(i) <= 1) {
      pts.push_back(pc.col(i));
      if (vec(i) <= min_dis) {
        min_pt_id = cnt;
        min_dis = vec(i);
      }
      cnt++;
    }
  }
  if (!pts.empty()) {
    // out.resize(3, pts.size());
    // for (size_t i = 0; i < pts.size(); i++) {
    //   out.col(i) = pts[i];
    // }
    out = Eigen::Map<const Eigen::Matrix<float, 3, -1, Eigen::ColMajor>>(pts[0].data(), 3, pts.size());
    return true;
  } else {
    return false;
  }
}// //}

bool Ellipsoid::inside(const Eigen::Vector3f &pt) const {// //{
  return dist(pt) <= 1;
}// //}
