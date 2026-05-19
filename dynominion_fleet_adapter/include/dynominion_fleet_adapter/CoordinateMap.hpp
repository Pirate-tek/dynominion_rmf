#ifndef DYNOMINION_FLEET_ADAPTER__COORDINATEMAP_HPP_
#define DYNOMINION_FLEET_ADAPTER__COORDINATEMAP_HPP_

#include <Eigen/Geometry>
#include <vector>
#include <iostream>
#include <stdexcept>

class CoordinateMap
{
public:
  CoordinateMap()
  {
    rmf_to_nav2_ = Eigen::Affine2d::Identity();
    nav2_to_rmf_ = Eigen::Affine2d::Identity();
  }

  static CoordinateMap from_translation(double dx, double dy, double dyaw = 0.0)
  {
    CoordinateMap map;
    map.rmf_to_nav2_ = Eigen::Translation2d(dx, dy) * Eigen::Rotation2D<double>(dyaw);
    map.nav2_to_rmf_ = map.rmf_to_nav2_.inverse();
    return map;
  }

  static CoordinateMap from_points(
    const std::vector<Eigen::Vector2d>& rmf_points,
    const std::vector<Eigen::Vector2d>& nav2_points)
  {
    if (rmf_points.size() != nav2_points.size() || rmf_points.size() < 3)
    {
      throw std::runtime_error("Need at least 3 points to compute affine transform");
    }

    CoordinateMap map;
    
    // Solve P_nav = M * P_rmf
    // We can use Eigen's least squares solver if there are more than 3 points
    Eigen::MatrixXd A(rmf_points.size() * 2, 6);
    Eigen::VectorXd b(rmf_points.size() * 2);

    for (size_t i = 0; i < rmf_points.size(); ++i)
    {
      // Row for x'
      A(i * 2, 0) = rmf_points[i].x();
      A(i * 2, 1) = rmf_points[i].y();
      A(i * 2, 2) = 1.0;
      A(i * 2, 3) = 0.0;
      A(i * 2, 4) = 0.0;
      A(i * 2, 5) = 0.0;
      b(i * 2) = nav2_points[i].x();

      // Row for y'
      A(i * 2 + 1, 0) = 0.0;
      A(i * 2 + 1, 1) = 0.0;
      A(i * 2 + 1, 2) = 0.0;
      A(i * 2 + 1, 3) = rmf_points[i].x();
      A(i * 2 + 1, 4) = rmf_points[i].y();
      A(i * 2 + 1, 5) = 1.0;
      b(i * 2 + 1) = nav2_points[i].y();
    }

    Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);
    
    Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();
    matrix(0, 0) = x(0);
    matrix(0, 1) = x(1);
    matrix(0, 2) = x(2);
    matrix(1, 0) = x(3);
    matrix(1, 1) = x(4);
    matrix(1, 2) = x(5);

    map.rmf_to_nav2_ = Eigen::Affine2d(matrix.block<2, 3>(0, 0));
    map.nav2_to_rmf_ = map.rmf_to_nav2_.inverse();
    
    return map;
  }

  Eigen::Vector3d rmf_to_nav2(const Eigen::Vector3d& rmf_pose) const
  {
    Eigen::Vector2d p_rmf = rmf_pose.head<2>();
    Eigen::Vector2d p_nav2 = rmf_to_nav2_ * p_rmf;
    
    double yaw_rmf = rmf_pose.z();
    double rotation = Eigen::Rotation2D<double>(rmf_to_nav2_.rotation()).angle();
    double yaw_nav2 = yaw_rmf + rotation;
    
    return Eigen::Vector3d(p_nav2.x(), p_nav2.y(), yaw_nav2);
  }

  Eigen::Vector3d nav2_to_rmf(const Eigen::Vector3d& nav2_pose) const
  {
    Eigen::Vector2d p_nav2 = nav2_pose.head<2>();
    Eigen::Vector2d p_rmf = nav2_to_rmf_ * p_nav2;
    
    double yaw_nav2 = nav2_pose.z();
    double rotation = Eigen::Rotation2D<double>(nav2_to_rmf_.rotation()).angle();
    double yaw_rmf = yaw_nav2 + rotation;
    
    return Eigen::Vector3d(p_rmf.x(), p_rmf.y(), yaw_rmf);
  }

private:
  Eigen::Affine2d rmf_to_nav2_;
  Eigen::Affine2d nav2_to_rmf_;
};

#endif // DYNOMINION_FLEET_ADAPTER__COORDINATEMAP_HPP_
