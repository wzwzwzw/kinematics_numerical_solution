// File:          human_gait.cpp
// Date:
// Description:
// Author:
// Modifications:
#include <PinocchioInterface.h>
#include <human_kinematics.h>
#include <human_leg_solveQP.h>

#include <iostream>
#include <webots/Robot.hpp>
#include <webots/motor.h>
#include <webots/robot.h>

#include "human_kinematics.h"
#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"
#include "webotsInterface.h"
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sstream>
#include <std_msgs/Float64.h>
#include <stdlib.h>
#include <string>
#include <thread>
#include <time.h>
#include <vector>

#include <filesystem>
// #include <iostream>

using namespace webots;
WebotsRobot humanWebots;
sim_human_kinematics::human_kinematics Human_kinematics;
rosgraph_msgs::Clock useSimTime; // use sim time
ros::Time rosTimeSim(0, 0);      // for stamp-time
ros::Duration stepTime_Sim(0, 2e6);

sensor_msgs::Imu imu_body;
geometry_msgs::Pose odome_data;
sensor_msgs::JointState jnt_feed;

int main(int argc, char **argv) {

  std::string current_path = std::filesystem::current_path().string();
  // std::cout << "Current path: " << current_path << std::endl;

  std::string urdf_filename = "../../URDF/urdf_ugot_human.urdf";

  leg_QP leg_qp_l(urdf_filename);
  leg_QP leg_qp_r(urdf_filename);
  PinocchioInterface Pinocchio_kinematics(urdf_filename);

  ros::init(argc, argv, "webots_human", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  ros::Publisher webots_sim_time_pub =
      n.advertise<rosgraph_msgs::Clock>("/clock", 1);
  humanWebots.initWebots();

  Eigen::VectorXd q(17);
  q << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  Eigen::VectorXd pose =
      Pinocchio_kinematics.computeEndEffectorPose(q, "left_ankle_roll_joint");
  std::cout << "End Effector lfoot_Pose: " << pose.transpose() << std::endl;
  pose =
      Pinocchio_kinematics.computeEndEffectorPose(q, "right_ankle_roll_joint");
  std::cout << "End Effector rfoot_Pose: " << pose.transpose() << std::endl;

  Eigen::VectorXd joint_q_left(10), joint_q_right(10), pos_des_left(6),
      pos_des_right(6), base_des(6), q_real_series(10);
  joint_q_left.setZero();
  joint_q_right.setZero();
  pos_des_left.setZero();
  pos_des_right.setZero();

  Human_kinematics.cstModeFlag << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  Human_kinematics.jntTauCmd.setConstant(0.0);
  Human_kinematics.jntPosCmd << 0, -0.533345, 1.06669, -0.533345, 0, 0,
      -0.533345, 1.06669, -0.533345, 0;

  Eigen::VectorXd Q = Eigen::VectorXd::Zero(SENSER_NUM);

  while (ros::ok) {
    humanWebots.robot->step(TIME_STEP);
    Human_kinematics.wbTime = humanWebots.robot->getTime();
    // if (Human_kinematics.wbTime >= 2) {
    //   Human_kinematics.wbTime = 2;
    // }

    q_real_series = humanWebots.getMotorPos();

    base_des << 0, 0, 0, 0, 0, 0;
    pos_des_left << 0, 0.0543,
        -0.2494 + 0.02 +
            abs(0.02 * sin(2 * M_PI / 8 * Human_kinematics.wbTime)),
        0, -0, 0;
    pos_des_right << 0, -0.0543,
        -0.2494 + 0.02 +
            abs(0.02 * sin(2 * M_PI / 8 * Human_kinematics.wbTime)),
        0, -0, 0;

    joint_q_left =
        leg_qp_l.sovle_qp(LEFT, pos_des_left, base_des, q_real_series);
    joint_q_right =
        leg_qp_r.sovle_qp(RIGHT, pos_des_right, base_des, q_real_series);

    // std::cout << "q_real_series:\n" << q_real_series << std::endl;

    Human_kinematics.jntPosCmd << joint_q_left[0], joint_q_left[1],
        joint_q_left[2], joint_q_left[3], joint_q_left[4], joint_q_right[0],
        joint_q_right[1], joint_q_right[2], joint_q_right[3], joint_q_right[4];

    humanWebots.setMotorState(Human_kinematics.jntPosCmd,
                              Human_kinematics.jntTauCmd,
                              Human_kinematics.cstModeFlag);

    rosTimeSim += stepTime_Sim;
    useSimTime.clock += stepTime_Sim;
    if (useSimTime.clock.nsec >= 1e9) {
      useSimTime.clock.nsec -= 1e9;
      useSimTime.clock.sec += 1;
    }
    jnt_feed.header.stamp = rosTimeSim;
    imu_body.header.stamp = rosTimeSim;

    webots_sim_time_pub.publish(useSimTime);

    ros::spinOnce();
    // loop_rate.sleep();
  };
  humanWebots.deleteRobot();
  return 0;
}
