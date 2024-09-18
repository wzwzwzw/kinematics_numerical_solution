#include "human_kinematics.h"

namespace sim_human_kinematics {
human_kinematics::human_kinematics() {
  cstModeFlag = Eigen::VectorXd::Zero(SERVO_NUM);
  jntPosCmd = Eigen::VectorXd::Zero(SERVO_NUM);
  jntTauCmd = Eigen::VectorXd::Zero(SERVO_NUM);

  vector_left_Aoutside << 0, 56.39 * 0.001,
      314.00 * 0.001; //把A点迁移到与B点平行
  vector_left_Ainside << 0, -57.01 * 0.001, 202.00 * 0.001;
  vector_right_Aoutside << 0, -56.39 * 0.001, 314.00 * 0.001;
  vector_right_Ainside << 0, 57.01 * 0.001, 202.00 * 0.001;

  vector_left_Boutside << -65.78 * 0.001, 56.39 * 0.001, 337.94 * 0.001;
  vector_left_Binside << -65.78 * 0.001, -57.01 * 0.001, 225.94 * 0.001;
  vector_right_Boutside << -65.78 * 0.001, -56.39 * 0.001, 337.94 * 0.001;
  vector_right_Binside << -65.78 * 0.001, 57.01 * 0.001, 225.94 * 0.001;

  vector_left_Coutside << -65.46 * 0.001, 56.70 * 0.001, 23.94 * 0.001;
  vector_left_Cinside << -65.47 * 0.001, -56.70 * 0.001, 23.94 * 0.001;
  vector_right_Coutside << -65.46 * 0.001, -56.70 * 0.001, 23.94 * 0.001;
  vector_right_Cinside << -65.47 * 0.001, 56.70 * 0.001, 23.94 * 0.001;

  l_rod_long = 314 * 0.001; //需要转化为m吧
  l_rod_short = 202 * 0.001;
  l_bar_long = 70.00 * 0.001;
  l_bar_short = 70.00 * 0.001;

  theta0_left_outside = 19.99849 * M_PI / 180;
  theta0_left_inside = 19.99849 * M_PI / 180;
  theta0_right_outside = 19.99849 * M_PI / 180;
  theta0_right_inside = 19.99849 * M_PI / 180;
}
human_kinematics::~human_kinematics() {}

Eigen::Vector2d human_kinematics::joint2motor(Eigen::Vector2d angle, int flag) {
  Eigen::Vector2d motor_angle; // 0为外侧电机，1为内侧电机

  double roll = angle[1];
  double pitch = angle[0]; // angle 先pitch再roll
  Eigen::Matrix3d X_rot;
  Eigen::Vector3d r_c1, r_c2, r_A1, r_A2;
  double a, b, c;
  X_rot = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  if (flag == 1) {
    //左脚
    r_c1 = X_rot * vector_left_Coutside;
    r_A1 = vector_left_Aoutside;
    a = (r_c1 - r_A1)[0];
    b = (r_c1 - r_A1)[2];
    c = (pow(l_rod_long, 2) - pow(l_bar_long, 2) -
         (r_c1 - r_A1).squaredNorm()) /
        (2 * l_bar_long);
    motor_angle[0] = asin((-(b * c) + sqrt(b * b * c * c -
                                           (a * a + b * b) * (c * c - a * a))) /
                          (a * a + b * b)) -
                     theta0_left_outside;
    r_c2 = X_rot * vector_left_Cinside;
    r_A2 = vector_left_Ainside;
    a = (r_c2 - r_A2)[0];
    b = (r_c2 - r_A2)[2];
    c = (pow(l_rod_short, 2) - pow(l_bar_short, 2) -
         (r_c2 - r_A2).squaredNorm()) /
        (2 * l_bar_short);
    motor_angle[1] = asin((-(b * c) + sqrt(b * b * c * c -
                                           (a * a + b * b) * (c * c - a * a))) /
                          (a * a + b * b)) -
                     theta0_left_inside;
  } else if (flag == 2) {
    //右脚
    r_c1 = X_rot * vector_right_Coutside;
    r_A1 = vector_right_Aoutside;
    a = (r_c1 - r_A1)[0];
    b = (r_c1 - r_A1)[2];
    c = (pow(l_rod_long, 2) - pow(l_bar_long, 2) -
         (r_c1 - r_A1).squaredNorm()) /
        (2 * l_bar_long);
    motor_angle[0] = asin((-(b * c) + sqrt(b * b * c * c -
                                           (a * a + b * b) * (c * c - a * a))) /
                          (a * a + b * b)) -
                     theta0_right_outside;
    r_c2 = X_rot * vector_right_Cinside;
    r_A2 = vector_right_Ainside;
    a = (r_c2 - r_A2)[0];
    b = (r_c2 - r_A2)[2];
    c = (pow(l_rod_short, 2) - pow(l_bar_short, 2) -
         (r_c2 - r_A2).squaredNorm()) /
        (2 * l_bar_short);
    motor_angle[1] = asin((-(b * c) + sqrt(b * b * c * c -
                                           (a * a + b * b) * (c * c - a * a))) /
                          (a * a + b * b)) -
                     theta0_right_inside;
  }

  return motor_angle;
}
Eigen::Matrix<double, 2, 2> human_kinematics::get_joint2motor_Jacobian(
    Eigen::Vector2d joint_angle, Eigen::Vector2d motor_angle, int flag_ankle) {
  Eigen::Matrix<double, 2, 6> J_joint;
  Eigen::Matrix<double, 2, 2> J_motor;
  Eigen::Matrix<double, 6, 2> G;
  Eigen::Matrix<double, 2, 2> J_joint2motor;
  Eigen::Vector3d r_B1, r_B2, r_C1, r_C2; // 1 outside 2 inside
  Eigen::Vector3d s1, s2;
  Eigen::Vector3d r_BC1, r_BC2; // r_bc=r_rod
  Eigen::Vector3d r_AB1, r_AB2; // r_ab=r_bar
  s1 << 0, 1, 0;
  s2 << 0, 1, 0;

  double roll = joint_angle[1];
  double pitch = joint_angle[0]; // angle 先pitch再roll
  Eigen::Matrix3d X_rot, R_y_outside, R_y_inside;
  X_rot = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  R_y_outside = Eigen::AngleAxisd(
      motor_angle[0],
      Eigen::Vector3d::UnitY()); // motor_angle 0 outside 1 inside
  R_y_inside = Eigen::AngleAxisd(motor_angle[1], Eigen::Vector3d::UnitY());

  if (flag_ankle == 1) {
    //左脚
    r_B1 = vector_left_Aoutside +
           R_y_outside * (vector_left_Boutside - vector_left_Aoutside);
    r_C1 = X_rot * vector_left_Coutside;
    r_BC1 = r_C1 - r_B1;
    r_AB1 = r_B1 - vector_left_Aoutside;

    r_B2 = vector_left_Ainside +
           R_y_inside * (vector_left_Binside - vector_left_Ainside);
    r_C2 = X_rot * vector_left_Cinside;
    r_BC2 = r_C2 - r_B2;
    r_AB2 = r_B2 - vector_left_Ainside;

  } else if (flag_ankle == 2) {
    //右脚
    r_B1 = vector_right_Aoutside +
           R_y_outside * (vector_right_Boutside - vector_right_Aoutside);
    r_C1 = X_rot * vector_right_Coutside;
    r_BC1 = r_C1 - r_B1;
    r_AB1 = r_B1 - vector_right_Aoutside;

    r_B2 = vector_right_Ainside +
           R_y_inside * (vector_right_Binside - vector_right_Ainside);
    r_C2 = X_rot * vector_right_Cinside;
    r_BC2 = r_C2 - r_B2;
    r_AB2 = r_B2 - vector_right_Ainside;
  }

  J_joint.block<1, 3>(0, 0) =
      r_BC1.transpose(); // 将 r_A1B1 的三个分量映射到 J_joint 的前三列
  J_joint.block<1, 3>(1, 0) = r_BC2.transpose();
  J_joint.block<1, 3>(0, 3) = (r_C1.cross(r_BC1)).transpose();
  J_joint.block<1, 3>(1, 3) = (r_C2.cross(r_BC2)).transpose();

  J_motor << s1.transpose() * (r_AB1.cross(r_BC1)), 0, 0,
      s2.transpose() * (r_AB2.cross(r_BC2));

  G.block<3, 1>(0, 0) << 0, 0, 0;
  G.block<3, 1>(0, 1) << 0, 0, 0;
  G.block<3, 1>(3, 0) << 0, 1, 0;
  G.block<3, 1>(3, 1) << std::cos(pitch), 0,
      -std::sin(
          pitch); // std::sin(pitch)应该加负号
                  // 原论文是先roll后pitch解的G，但是自己定义的x都是先pitch后roll，所以(3,0)和(3,1)需要反过来，此处杨平的资料和琛哥的资料有误

  J_joint2motor = J_motor.inverse() * J_joint * G;
  return J_joint2motor;
}

Eigen::Vector2d human_kinematics::motor2joint(
    Eigen::Vector2d motor_des, Eigen::Vector2d joint_real,
    int flag_ankle) { // motor_des 0 outside 1 inside joint_real 0 pitch 1 roll
  // joint_real为上一时刻的joint角度
  // joint_angle为给定motor_des（与零位相减的绝对值）时，所要求的值

  Eigen::Vector2d joint_angle;
  Eigen::Vector2d error_iterate;
  Eigen::Vector2d x_k;
  Eigen::Vector2d x_knew;
  double pitch_des = joint_real[0];
  double roll_des = joint_real[1];

  Eigen::Vector2d motor_real = joint2motor(joint_real, flag_ankle);
  //   std::cout << "motor_real: \n" << motor_real << std::endl;

  double error_motor1 = motor_real[0] - motor_des[0];
  double error_motor2 = motor_real[1] - motor_des[1];
  const double eps = 1e-8; //-8 -12
  const int IT_MAX = 1000; // 1000
  int i = 0;
  Eigen::Matrix<double, 2, 2> J_inverse;
  Eigen::Matrix3d rotation;
  Eigen::Matrix3d rotation_pitch_trans;
  Eigen::Vector3d endpoint_1;
  Eigen::Vector3d endpoint_2;

  x_k << joint_real[0], joint_real[1];
  //   std::cout << "x_k: \n" << x_k << std::endl;
  while (true) {
    if (((abs(error_motor1) <= eps) and (abs(error_motor2) <= eps)) or
        (i >= IT_MAX)) {
      joint_angle << pitch_des, roll_des;
      if (i >= IT_MAX) {
        std::cout << "error_motor1: " << error_motor1 << std::endl;
        std::cout << "error_motor2: " << error_motor2 << std::endl;
      }
      break;
    }
    // std::cout << "x_k: \n" << x_k << std::endl;
    pitch_des = x_k[0];
    roll_des = x_k[1];
    motor_real = joint2motor(x_k, flag_ankle);
    J_inverse = get_joint2motor_Jacobian(x_k, motor_real, flag_ankle).inverse();
    error_iterate << motor_real[0] - motor_des[0], motor_real[1] - motor_des[1];
    // std::cout << "error_iterate: \n" << error_iterate << std::endl;
    // std::cout << "motor_real: \n" << motor_real << std::endl;
    // std::cout << "motor_des: \n" << motor_des << std::endl;

    // std::cout << "J_inverse: " << J_inverse << std::endl;
    // std::cout<<"J_transpose:
    // "<<get_ankle_Jacobian(flag_ankle,endpoint_1,endpoint_2,pitch_des).transpose()<<std::endl;
    // std::cout<<"J:
    // "<<get_ankle_Jacobian(flag_ankle,endpoint_1,endpoint_2,pitch_des)<<std::endl;
    x_knew = x_k - J_inverse * error_iterate;
    // std::cout << "x_knew: \n" << x_knew << std::endl;

    error_motor1 = error_iterate[0];
    error_motor2 = error_iterate[1];
    x_k = x_knew;
    i++;
  }
  // std::cout<<"iterate_i: "<<i<<std::endl;

  return joint_angle;
}

} // namespace sim_human_kinematics