#include "webotsInterface.h"
#include "webots/position_sensor.h"
#include <fstream>
#include <iostream>

using namespace webots;

void WebotsRobot::initWebots() {
  legMotor.resize(SERVO_NUM);
  legMotor[0] = robot->getMotor("left_hip_roll_joint");
  legMotor[1] = robot->getMotor("left_hip_pitch_joint");
  legMotor[2] = robot->getMotor("left_knee_pitch_joint");
  legMotor[3] = robot->getMotor("left_ankle_pitch_joint");
  legMotor[4] = robot->getMotor("left_ankle_roll_joint");
  legMotor[5] = robot->getMotor("right_hip_roll_joint");
  legMotor[6] = robot->getMotor("right_hip_pitch_joint");
  legMotor[7] = robot->getMotor("right_knee_pitch_joint");
  legMotor[8] = robot->getMotor("right_ankle_pitch_joint");
  legMotor[9] = robot->getMotor("right_ankle_roll_joint");

  for (int i = 0; i < SERVO_NUM; i++) {
    legMotor[i]->setControlPID(90, 0, 0.2);
  }

  legPosSensor.resize(SENSER_NUM);
  legPosSensor[0] = robot->getPositionSensor("left_hip_roll_joint_sensor");
  legPosSensor[1] = robot->getPositionSensor("left_hip_pitch_joint_sensor");
  legPosSensor[2] = robot->getPositionSensor("left_knee_pitch_joint_sensor");

  legPosSensor[3] = robot->getPositionSensor("left_ankle_pitch_joint_sensor");
  legPosSensor[4] = robot->getPositionSensor("left_ankle_roll_joint_sensor");
  legPosSensor[5] = robot->getPositionSensor("right_hip_roll_joint_sensor");

  legPosSensor[6] = robot->getPositionSensor("right_hip_pitch_joint_sensor");
  legPosSensor[7] = robot->getPositionSensor("right_knee_pitch_joint_sensor");
  legPosSensor[8] = robot->getPositionSensor("right_ankle_pitch_joint_sensor");
  legPosSensor[9] = robot->getPositionSensor("right_ankle_roll_joint_sensor");

  // IMU = robot->getInertialUnit("Body_IMU_Angle");
  // Gyro_torso = robot->getGyro("Body_IMU_AngleVelocity");
  // accelerometer = robot->getAccelerometer("Body_IMU_Acceleration");
  // GPS_torso = robot->getGPS("GPS_Torso");

  // enable
  for (int i = 0; i < SERVO_NUM; i++) {
    legMotor[i]->enableTorqueFeedback(TIME_STEP);
  }

  for (int i = 0; i < SENSER_NUM; i++) {
    legPosSensor[i]->enable(TIME_STEP);
  }

  //   IMU->enable(TIME_STEP);
  //   Gyro_torso->enable(TIME_STEP);
  //   GPS_torso->enable(TIME_STEP);
  //   accelerometer->enable(TIME_STEP);
}

bool WebotsRobot::setMotorState(const Eigen::VectorXd &jointPosTar,
                                const Eigen::VectorXd &jointTauTar,
                                const Eigen::VectorXd &jointCtrlMode) {
  double torque;
  for (int i = 0; i < SERVO_NUM; i++) {
    if (jointCtrlMode[i] == 0) {
      legMotor[i]->setPosition(jointPosTar[i]);
    }
    if (jointCtrlMode[i] == 1) {
      torque = jointTauTar(i, 0);
      legMotor[i]->setTorque(torque);
    }
  }
  return true;
}

/*
 * Description:   Set the motor torque to control ubt_quad
 */
bool WebotsRobot::setMotorTau(const Eigen::VectorXd &jointTorTar) {
  double torque;
  for (int i = 0; i < SERVO_NUM; i++) {
    torque = jointTorTar(i, 0);
    legMotor[i]->setTorque(torque);
  }

  return true;
}

bool WebotsRobot::setMotorPos(const Eigen::VectorXd &jointPosTar) {
  for (int i = 0; i < SERVO_NUM; i++) {
    legMotor[i]->setPosition(jointPosTar(i, 0));
  }
  return true;
}

Eigen::VectorXd WebotsRobot::getMotorPos() {
  Eigen::VectorXd Q = Eigen::VectorXd::Zero(SENSER_NUM);
  for (int i = 0; i < SENSER_NUM; i++) {
    Q(i, 0) = legPosSensor[i]->getValue();
  }
  return Q;
}

Eigen::VectorXd WebotsRobot::getMotorTau() {
  Eigen::VectorXd Tau = Eigen::VectorXd::Zero(SERVO_NUM);
  for (int i = 0; i < SERVO_NUM; i++) {
    Tau(i, 0) = legMotor[i]->getTorqueFeedback();
  }
  return Tau;
}

// Eigen::Vector3d WebotsRobot::getTorsoPosition() {
//   const double *data = GPS_torso->getValues();
//   Eigen::Vector3d pos(data[0], data[1], data[2]);

//   return pos;
// }

// Eigen::Vector3d WebotsRobot::getTorsoAcc() {
//   const double *data = accelerometer->getValues();
//   Eigen::Vector3d acceleration(data[2], data[0], data[1]);
//   return acceleration;
// }

// Eigen::Vector3d WebotsRobot::getTorsoAngularVelocity() {
//   const double *data = Gyro_torso->getValues();
//   Eigen::Vector3d eulerAngleRate(data[0], data[1], data[2]);
//   return eulerAngleRate;
// }

// Eigen::Vector3d WebotsRobot::getTorsoAngular() {
//   const double *data = IMU->getRollPitchYaw();
//   Eigen::Vector3d eulerAngle(data[0], data[1], data[2]);
//   return eulerAngle;
// }

void WebotsRobot::deleteRobot() { delete robot; }
