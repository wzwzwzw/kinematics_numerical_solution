#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <qpOASES.hpp>
#include <string>

#define LEFT 1
#define RIGHT 2

class leg_QP {
public:
  leg_QP(const std::string &urdf_filename);
  ~leg_QP();
  Eigen::VectorXd sovle_qp(int flag, Eigen::VectorXd pos_des,
                           Eigen::VectorXd base_des,
                           Eigen::VectorXd jointq_now);

private:
  pinocchio::Model model;
  pinocchio::Data data;
  qpOASES::SQProblem qp; // nv 和A的行数
  qpOASES::Options options;
  int joint_num;
  double error;
  std::string urdf_name;
  // x y z r p y
  Eigen::VectorXd get_g_matrix(Eigen::VectorXd pos_des, Eigen::VectorXd pos_now,
                               pinocchio::Data::Matrix6x J_now);
  Eigen::MatrixXd get_H_matrix(pinocchio::Data::Matrix6x J_now);
  Eigen::VectorXd get_e_matrix();
  Eigen::MatrixXd get_A_matrix(int flag);
  Eigen::Vector3d rotationToEuler(Eigen::Matrix3d rotation_matrix);
  Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw);
};
