#ifndef YOUBOT_ARM_JACOBI_CONTROL_VELOCITY_CONTROLLER_H
#define YOUBOT_ARM_JACOBI_CONTROL_VELOCITY_CONTROLLER_H

#include <iostream>
#include <fstream>
#include <vector>
#include <stdlib.h>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include <boost/units/io.hpp>
#include <boost/units/systems/si/angular_velocity.hpp>
#include <boost/units/systems/si/velocity.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/QR>
#include<Eigen/Eigenvalues>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointPositions.h>

#include <youbot_arm_kinematics/kinematics.h>
#include <visp3/core/vpEigenConversion.h>
#include <visp3/core/vpMatrix.h>


using namespace std;
using namespace Eigen;

class VelocityController {
  static const int NUMBER_OF_JOINTS = 5; /* FIXME: may be get me from ks, maan? */
  Kinematics ks;
  double kp, kd, ki;
  Matrix<double, 6, 6> K; /* !!! Init in contructor. Uses for controller. */
    
  ros::NodeHandle nh;
  ros::Publisher arm_position_pub;
  ros::Publisher arm_velocity_pub;
  ros::Subscriber arm_js_sub;
  ros::Subscriber joy_sub;

  /* trajectory point */
  Vector6d s_d;
  Vector6d ds_d;

  std::ofstream out;
  int counter;
  Vector6d ei;
  Vector6d ed;
  Vector6d last_e;
  Vector6d u;
  Matrix<double, 6, 1> dq0;
  Matrix<double, 6, 5> JprevWell;

  void js_callback(const sensor_msgs::JointState &msg);
  void js_callback1(const sensor_msgs::JointState &msg);
  void joy_callback(const sensor_msgs::Joy &msg);


public:
  VelocityController();
  VelocityController(Kinematics _ks);
  VelocityController(Kinematics _ks, double _kp, double _kd, double _ki);
  ~VelocityController();

  void work();
};

#endif //YOUBOT_ARM_JACOBI_CONTROL_VELOCITY_CONTROLLER_H
