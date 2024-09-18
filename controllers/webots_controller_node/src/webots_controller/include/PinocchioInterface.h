// KinematicsInterface.h
#pragma once

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include <Eigen/Dense>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <string>

class PinocchioInterface {
public:
  PinocchioInterface(const std::string &urdf_filename);
  ~PinocchioInterface();
  // Eigen::VectorXd q;
  const pinocchio::SE3 oMdes;
  // pinocchio::Data::Matrix6x J;

  Eigen::VectorXd computeEndEffectorPose(const Eigen::VectorXd &q,
                                         const std::string &joint_name,
                                         bool use_xyz = false);
  Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw);

private:
  pinocchio::Model model;
  pinocchio::Data data;
  Eigen::VectorXd q;
  Eigen::VectorXd v;
  Eigen::VectorXd a;
  Eigen::VectorXd joint_old;
  Eigen::VectorXd joint_v;
};
