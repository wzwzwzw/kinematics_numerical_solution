#include <human_leg_solveQP.h>

leg_QP::leg_QP(const std::string &urdf_filename) : qp(0, 0) {
  pinocchio::urdf::buildModel(urdf_filename, model);
  joint_num = model.nv - 6;
  qp = qpOASES::SQProblem(model.nv, model.nv);
}
leg_QP::~leg_QP() {}

Eigen::Vector3d leg_QP::rotationToEuler(Eigen::Matrix3d rotation_matrix) {
  double roll, pitch, yaw;
  // 根据旋转矩阵的定义，可以通过一些三角函数来计算欧拉角
  if (rotation_matrix(2, 0) < 1) {
    if (rotation_matrix(2, 0) > -1) {
      // 计算 pitch
      pitch = asin(-rotation_matrix(2, 0));
      // 计算 roll 和 yaw
      roll = atan2(rotation_matrix(2, 1), rotation_matrix(2, 2));
      yaw = atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));
    } else {
      // 当 rotation_matrix(2, 0) == -1 时，pitch = -pi/2
      pitch = -M_PI / 2;
      // 由于 pitch = -pi/2，因此 roll 和 yaw 的和为 atan2(-r21, r22)
      roll = -atan2(rotation_matrix(1, 2), rotation_matrix(1, 1));
      yaw = 0;
    }
  } else {
    // 当 rotation_matrix(2, 0) == 1 时，pitch = pi/2
    pitch = M_PI / 2;
    // 由于 pitch = pi/2，因此 roll 和 yaw 的差为 atan2(r21, r22)
    roll = atan2(rotation_matrix(1, 2), rotation_matrix(1, 1));
    yaw = 0;
  }

  // 将欧拉角存储到 Vector3d 中并返回
  return Eigen::Vector3d(roll, pitch, yaw);
}

Eigen::Quaterniond leg_QP::eulerToQuaternion(double roll, double pitch,
                                             double yaw) {
  // Eigen中的角度需要转换为弧度
  // double r = roll * M_PI / 180;
  // double p = pitch * M_PI / 180;
  // double y = yaw * M_PI / 180;

  // 弧度
  double r = roll;
  double p = pitch;
  double y = yaw;

  // 使用 Eigen 的 AngleAxis 初始化四元数
  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());

  return q;
}

Eigen::MatrixXd leg_QP::get_H_matrix(pinocchio::Data::Matrix6x J_now) {
  Eigen::MatrixXd JtJ = Eigen::MatrixXd::Zero(model.nv, model.nv);
  JtJ = J_now.transpose() * J_now; // H=2*JT*J
  return JtJ;
}

Eigen::VectorXd leg_QP::get_g_matrix(Eigen::VectorXd pos_des,
                                     Eigen::VectorXd pos_now,
                                     pinocchio::Data::Matrix6x J_now) {
  Eigen::VectorXd e(6);
  Eigen::VectorXd g = Eigen::VectorXd::Zero(model.nv); // g=-2*JT*e
  e = pos_des - pos_now;
  // std::cout<<"error: "<<e<<std::endl;
  g = -J_now.transpose() * e;
  return g;
}

Eigen::MatrixXd leg_QP::get_A_matrix(int flag) {
  Eigen::MatrixXd A(model.nv, model.nv);
  A.setIdentity();
  if (flag == 1) {
    A.coeffRef(6, 6) = 0;
    A.coeffRef(7, 7) = 0;
    A.coeffRef(8, 8) = 0;
    A.coeffRef(9, 9) = 0;
    A.coeffRef(10, 10) = 0;

  } else {
    A.coeffRef(11, 11) = 0;
    A.coeffRef(12, 12) = 0;
    A.coeffRef(13, 13) = 0;
    A.coeffRef(14, 14) = 0;
    A.coeffRef(15, 15) = 0;
  }
  return A;
}

Eigen::VectorXd leg_QP::get_e_matrix() {
  Eigen::VectorXd e = Eigen::VectorXd::Zero(model.nv); // A*q=e;
  return e;
}

Eigen::VectorXd
leg_QP::sovle_qp(int flag, Eigen::VectorXd pos_des, Eigen::VectorXd base_des,
                 Eigen::VectorXd jointq_now) { //为啥不是imu的浮动基座的位置
  data = pinocchio::Data(model);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
  Eigen::Quaterniond q_quat;
  q_quat = eulerToQuaternion(base_des[3], base_des[4], base_des[5]);

  q << base_des[0], base_des[1], base_des[2], q_quat.x(), q_quat.y(),
      q_quat.z(), q_quat.w(), jointq_now[0], jointq_now[1], jointq_now[2],
      jointq_now[3], jointq_now[4], jointq_now[5], jointq_now[6], jointq_now[7],
      jointq_now[8], jointq_now[9];

  // q <<  0,0,0, 0,0,0,1,   0,0,0,0,0,0,0,   0,0,0,0,0,0,0;
  pinocchio::Data::Matrix6x J(6, model.nv);
  int JOINT_ID;
  int nWSR;
  Eigen::VectorXd pos_now(6);
  if (flag == 1) { // 左腿
    JOINT_ID = model.getJointId("left_ankle_roll_joint");
    nWSR = 1000;
    // std::cout << "left left" << std::endl;
  } else if (flag == 2) {
    JOINT_ID = model.getJointId("right_ankle_roll_joint");
    nWSR = 100000;
  }
  pinocchio::computeJointJacobian(model, data, q, JOINT_ID, J);
  // std::cout<<"model.nv: "<<model.nv<<std::endl;
  // std::cout<<"model.nq: "<<model.nq<<std::endl;
  // std::cout<<"J: \n"<<J<<std::endl;
  Eigen::MatrixXd H = get_H_matrix(J);
  // std::cout<<"H: \n"<<H<<std::endl;
  pinocchio::forwardKinematics(model, data, q);
  const pinocchio::SE3 &endPose = data.oMi[JOINT_ID];
  Eigen::Vector3d end_pose = endPose.translation();
  Eigen::Vector3d end_euler = rotationToEuler(endPose.rotation());
  pos_now << end_pose, end_euler;
  Eigen::VectorXd g = get_g_matrix(pos_des, pos_now, J);
  // std::cout<<"g: \n"<<g<<std::endl;
  Eigen::MatrixXd A = get_A_matrix(flag);
  // std::cout<<"A: \n"<<A<<std::endl;
  Eigen::VectorXd e = get_e_matrix();
  // std::cout<<"e: \n"<<e<<std::endl;
  Eigen::VectorXd lbA = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd ubA = Eigen::VectorXd::Zero(model.nv);
  lbA = e;
  ubA = e;
  Eigen::VectorXd lb = Eigen::VectorXd::Constant(
      model.nv, -1.0); // 为啥会影响计算？有限位再考虑下面的限制方式
  Eigen::VectorXd ub = Eigen::VectorXd::Constant(model.nv, 1.0);
  // Eigen::VectorXd lb = Eigen::VectorXd::Zero(model.nv);
  // Eigen::VectorXd ub = Eigen::VectorXd::Zero(model.nv);
  // lb <<-1,-1,-1,-1,-1,-1,
  //
  // -3.14 - jointq_now[0], -3.14 - jointq_now[1], -3.14 - jointq_now[2], -3.14
  // - jointq_now[3], -3.14 - jointq_now[4],
  //     -3.14 - jointq_now[5], -3.14 - jointq_now[6],
  //     //
  //     -3.14 - jointq_now[7], -3.14 - jointq_now[8], -3.14 - jointq_now[9],
  //     -3.14 - jointq_now[10], -3.14 - jointq_now[11], -3.14 - jointq_now[12],
  //     -3.14 - jointq_now[13];
  // ub << 1,1,1,1,1,1,
  //     3.14-jointq_now[0],3.14-jointq_now[1],3.14-jointq_now[2],3.14-jointq_now[3],3.14-jointq_now[4],3.14-jointq_now[5],3.14-jointq_now[6],
  //     3.14-jointq_now[7],3.14-jointq_now[8],3.14-jointq_now[9],3.14-jointq_now[10],3.14-jointq_now[11],3.14-jointq_now[12],3.14-jointq_now[13];

  // std::cout<<"lbA: \n"<<lbA<<std::endl;
  // std::cout<<"ubA: \n"<<ubA<<std::endl;
  // std::cout<<"A:\n"<<A<<std::endl;
  // std::cout<<"lb: \n"<<lb<<std::endl;
  // std::cout<<"ub: \n"<<ub<<std::endl;

  // qpOASES::SQProblem qp(model.nv, model.nv); // nv 和A的行数
  // qpOASES::Options options;
  options.printLevel = qpOASES::PL_NONE; // 设置为 PL_NONE 以禁用输出信息
  qp.setOptions(options);

  qp.init(H.data(), g.data(), A.data(), lb.data(), ub.data(), lbA.data(),
          ubA.data(),
          nWSR); // 哪些需要转置还得注意一下
  Eigen::VectorXd qSolution(model.nv);
  qp.getPrimalSolution(qSolution.data());

  // std::cout<<"qSolution: \n"<<qSolution<<std::endl;
  Eigen::VectorXd q_result = Eigen::VectorXd::Zero(joint_num);
  q_result = qSolution.segment(6, joint_num);
  // std::cout<<"q_result: \n"<<q_result<<std::endl;
  Eigen::VectorXd q_result_true = Eigen::VectorXd::Zero(joint_num / 2.0);
  if (flag == 1) { // 设置的q的mimic关节也应该是一样的
    q_result_true << q_result[0] + jointq_now[0], q_result[1] + jointq_now[1],
        q_result[2] + jointq_now[2], q_result[3] + jointq_now[3],
        q_result[4] + jointq_now[4];
    // q_result_true <<
    // q_result[0],q_result[1],q_result[2],q_result[3],q_result[4],q_result[5],q_result[6];
  } else if (flag == 2) {
    q_result_true << q_result[5] + jointq_now[5], q_result[6] + jointq_now[6],
        q_result[7] + jointq_now[7], q_result[8] + jointq_now[8],
        q_result[9] + jointq_now[9];
  }
  // std::cout<<"q_result_true: "<<q_result_true<<std::endl;
  // if(flag==1){
  //     error=qSolution[10]-qSolution[9];
  // }
  // else{
  //     error=qSolution[16]-qSolution[17];
  // }

  // if((error<=0.0001) and  (error>=-0.0001)){
  // // std::cout<<"solve true!"<<std::endl;
  // }
  // else{
  //     if(flag==1){
  //         std::cout<<"solve left flase!"<<std::endl;
  //     }
  //     else{
  //         std::cout<<"solve right flase!"<<std::endl;
  //     }
  // }
  return q_result_true;
}
