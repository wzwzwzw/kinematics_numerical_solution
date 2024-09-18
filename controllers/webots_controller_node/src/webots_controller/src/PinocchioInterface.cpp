#include <PinocchioInterface.h>
#include <human_kinematics.h>

#include <chrono>
#include <iostream>

PinocchioInterface::PinocchioInterface(const std::string &urdf_filename) {
  pinocchio::urdf::buildModel(urdf_filename, model);
  data = pinocchio::Data(model);
  q = Eigen::VectorXd::Zero(model.nq);
  v = Eigen::VectorXd::Zero(model.nv);
  a = Eigen::VectorXd::Zero(model.nv);
  joint_old = Eigen::VectorXd::Zero(model.nq - 7);
  joint_v = Eigen::VectorXd::Zero(model.nq - 7);
}

PinocchioInterface::~PinocchioInterface() {}
Eigen::VectorXd PinocchioInterface::computeEndEffectorPose(
    const Eigen::VectorXd &q, const std::string &joint_name, bool use_xyz) {
  pinocchio::forwardKinematics(model, data, q);

  pinocchio::Data::Matrix6x J(6, model.nv);
  int JOINT_ID = model.getJointId(joint_name);
  pinocchio::computeJointJacobian(model, data, q, JOINT_ID, J);
  // std::cout<<"J_initial:\n"<<J<<std::endl;

  const pinocchio::SE3 &endEffectorPose =
      data.oMi[model.getJointId(joint_name)];
  Eigen::Vector3d euler_angles = endEffectorPose.rotation().eulerAngles(
      use_xyz ? 0 : 2, 1, use_xyz ? 2 : 0); // 旋转角ZYX
  Eigen::Vector3d translation_vector = endEffectorPose.translation();

  Eigen::VectorXd result(6);
  result << translation_vector, euler_angles;
  return result;
}
Eigen::Quaterniond
PinocchioInterface::eulerToQuaternion(double roll, double pitch, double yaw) {
  // 将欧拉角转换为四元数
  Eigen::Quaterniond q;
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  q.w() = cy * cp * cr + sy * sp * sr;
  q.x() = cy * cp * sr - sy * sp * cr;
  q.y() = sy * cp * sr + cy * sp * cr;
  q.z() = sy * cp * cr - cy * sp * sr;

  return q;
}
