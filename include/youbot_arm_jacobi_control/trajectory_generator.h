#ifndef YOUBOT_ARM_JACOBI_CONTROL_TRAJECTORY_GENERATOR_H
#define YOUBOT_ARM_JACOBI_CONTROL_TRAJECTORY_GENERATOR_H

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <youbot_arm_kinematics/kinematics.h>

struct s {
  double x, y, z, alpha;
};

class TrajectroyGenerator {
  double theta_s;
  double alpha_0;
  double Rc;

 public:
  TrajectroyGenerator();
  TrajectroyGenerator(double Rc, double theta_s, double alpha_0);
  void generate_js_trajectory(VectorNd qs, VectorNd qf, double t,
                              VectorNd dqs, VetorNd dqf, double v_max,
                              Matrix<double , Dynamic, 5> q,
                              Matrix<double , Dynamic, 5> dq,
                              Matrix<double , Dynamic, 5> ddq);

  s get_arc_for(double t, double v);

};

#endif //YOUBOT_ARM_JACOBI_CONTROL_TRAJECTORY_GENERATOR_H
