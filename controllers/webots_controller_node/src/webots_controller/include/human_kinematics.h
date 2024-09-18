#ifndef HUMAN_KINEMATICS_H
#define HUMAN_KINEMATICS_H

#include "webotsInterface.h"
#include <Eigen/Dense>
#include <cmath>

// namespace?
//改头文件名字，新增头文件对应cpp文件

namespace sim_human_kinematics {

class human_kinematics {
public:
  human_kinematics();
  ~human_kinematics();

  // kinematics parameter
  double delta_l;
  double cos_theta, delta_theta;
  double wbTime;
  double l1, l2, l3, initial_theta;
  double count{};
  Eigen::VectorXd cstModeFlag;
  Eigen::VectorXd jntPosCmd;
  Eigen::VectorXd jntTauCmd;

  double l_rod_long, l_rod_short, l_bar_long, l_bar_short;
  double theta0_left_outside, theta0_left_inside, theta0_right_outside,
      theta0_right_inside;
  Eigen::Vector3d vector_left_Aoutside, vector_left_Ainside,
      vector_right_Aoutside, vector_right_Ainside; // O to A
  Eigen::Vector3d vector_left_Boutside, vector_left_Binside,
      vector_right_Boutside, vector_right_Binside; // O to B
  Eigen::Vector3d vector_left_Coutside, vector_left_Cinside,
      vector_right_Coutside, vector_right_Cinside; // O to C

  // kinematics function processing
  //   double theta_transport(double delta_l);
  Eigen::Vector2d joint2motor(Eigen::Vector2d angle, int flag);
  Eigen::Matrix<double, 2, 2>
  get_joint2motor_Jacobian(Eigen::Vector2d joint_angle,
                           Eigen::Vector2d motor_angle, int flag_ankle);

  Eigen::Vector2d motor2joint(Eigen::Vector2d motor_des,
                              Eigen::Vector2d joint_des, int flag_ankle);

private:
};

} // namespace sim_human_kinematics
#endif