#ifndef WEBOTS_INTERFACE_H
#define WEBOTS_INTERFACE_H

#include <webots/Accelerometer.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/Node.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/motor.h>
#include <webots/robot.h>

#include <eigen3/Eigen/Dense>
#include <vector>

using namespace webots;

#ifndef TIME_STEP
#define TIME_STEP (2)
#endif // TIME_STEP

#ifndef SAMPLE_TIME
#define SAMPLE_TIME (0.002f)
#endif // SAMPLE_TIME

#define SERVO_NUM 10

#define SENSER_NUM 10

struct WebotState {
  Eigen::VectorXd jointPosAct = Eigen::VectorXd::Zero(SERVO_NUM);
  Eigen::VectorXd jointVelAct = Eigen::VectorXd::Zero(SERVO_NUM);
  Eigen::VectorXd jointTorAct = Eigen::VectorXd::Zero(SERVO_NUM);

  Eigen::VectorXd imu9DAct = Eigen::VectorXd::Zero(9);
  Eigen::VectorXd footGrfAct = Eigen::VectorXd::Zero(SERVO_NUM);

  Eigen::Vector3d torsoXyzAct = Eigen::Vector3d::Zero();
  Eigen::Vector3d torsoXyzVelAct = Eigen::Vector3d::Zero();
  Eigen::Vector3d torsoXyzAccAct = Eigen::Vector3d::Zero();

  Eigen::Vector3d torsoRpyAct = Eigen::Vector3d::Zero();
  Eigen::Vector3d torsoRpyVelAct = Eigen::Vector3d::Zero();
};

/**
 * @brief The WebotsRobot class
 */
class WebotsRobot {
public:
  // Robot *robot = new Robot();
  Supervisor *robot = new Supervisor();

  void initWebots();
  void deleteRobot();
  bool readData(double simTime, WebotState &robotState);
  bool setMotorPos(const Eigen::VectorXd &jointPosTar);
  bool setMotorTau(const Eigen::VectorXd &jointTauTar);
  bool setMotorState(const Eigen::VectorXd &jointPosTar,
                     const Eigen::VectorXd &jointTauTar,
                     const Eigen::VectorXd &jointCtrlMode);
  void inverse_kinematics(double &x_des, double &y_des,
                          Eigen::VectorXd &qExpand);
  void inverse_kinematics_new(double &x_des, double &y_des,
                              Eigen::VectorXd &qExpand);
  Eigen::VectorXd getMotorPos();
  Eigen::Vector3d getTorsoAngularVelocity();
  Eigen::Vector3d getTorsoAngular();
  Eigen::Vector3d getTorsoPosition();
  Eigen::VectorXd getMotorTau();
  void setMotorPid();

private:
  Eigen::Vector3d getTorsoAcc();
  Eigen::Vector3d getTorsoOrientation();
  Eigen::VectorXd getFootPosition();
  Eigen::VectorXd getFootForce();

  Eigen::Vector3d rotm2Rpy(const Eigen::Matrix3d &rotm);

  Eigen::Vector3d torsoXyzInit = Eigen::Vector3d::Zero();

  std::vector<Motor *> legMotor;
  std::vector<PositionSensor *> legPosSensor;
  std::vector<TouchSensor *> forceSensor;

  GPS *GPS_torso;
  InertialUnit *IMU;
  Gyro *Gyro_torso;
  Accelerometer *accelerometer;

  GPS *GPS_FLfoot;
  GPS *GPS_FLhip;
  Node *Torso;
};

#endif