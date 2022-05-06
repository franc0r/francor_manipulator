#ifndef MANIPULATORSIMPLEINVERSE_H_
#define MANIPULATORSIMPLEINVERSE_H_
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>

struct Axis {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

class ManipulatorSimpleInverse{
public:
  // ManipulatorSimpleInverse()
  // {

  // }
  // ~ManipulatorSimpleInverse() = default;

  static Axis compute2DForward(double x, double z)
  {
    // std::cout << "--------- 2d forward -----------" << std::endl;
    Axis axis;

    const double l0 = 0.38;
    const double l1 = 0.245;

    double ang_x = x + 0.6;
    // std::cout << "ang_x: " << ang_x << std::endl;
    double ang_z = (M_PI - (z + M_PI_4)) * -1;
    // std::cout << "ang_z: " << ang_z << std::endl;



    axis.x = l0 * std::cos(ang_x) + l1 * std::cos(ang_x + ang_z);
    axis.z = l0 * std::sin(ang_x) + l1 * std::sin(ang_x + ang_z);

    // std::cout << "axis.x: " << axis.x << std::endl;
    // std::cout << "axis.z: " << axis.z << std::endl;

    //offset for arm
    return axis;
  }
  static Axis compute2DInverse(double x, double z)
  {
    // std::cout << "--------- 2d inverse -----------" << std::endl;
    Axis axis;
    const double l0 = 0.38;
    const double l1 = 0.245;

    const auto l0_sq = l0 * l0;
    const auto l1_sq = l1 * l1;

    Eigen::Vector2d p(x, z);

    const double r_p = p.norm();
    const double r_p_sq = r_p * r_p;


    double alpha = std::acos((l1_sq - l0_sq - r_p_sq) / (-2 * l0 * r_p));

    double phi_p = std::atan2(p.y(), p.x());

    double gamma = std::acos((l1_sq + l0_sq - r_p_sq) / (2 * l0 * l1));

    axis.x = alpha + phi_p;// + 0.6; //todo check
    axis.z = gamma; // - M_PI_2; //todo check
    // std::cout << "axis.x: " << axis.x << std::endl;
    // std::cout << "axis.z: " << axis.z << std::endl;
    // std::cout << "-- transformed --" << std::endl;
    axis.x -= 0.6;
    axis.z -= M_PI_4;
    // std::cout << "axis.x: " << axis.x << std::endl;
    // std::cout << "axis.z: " << axis.z << std::endl;
    return axis;
  }
};

#endif  //MANIPULATORSIMPLEINVERSE_H_