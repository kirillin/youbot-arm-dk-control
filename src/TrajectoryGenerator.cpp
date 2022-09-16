#include "trajectory_genarator.h"

TrajectroyGenerator::TrajectroyGenerator() {}

TrajectroyGenerator(double Rc, double theta_s, double alpha_0): Rc(Rc), theta_s(theta_s), alpha_0(alpha_0) {
}

void TrajectroyGenerator::generate_js_trajectory(VectorNd qs, VectorNd qf, double t,
                                              VectorNd dqs, VectorNd dqf, double v_max,
                                              Matrix<double , Dynamic, 5> q,
                                              Matrix<double , Dynamic, 5> dq,
                                              Matrix<double , Dynamic, 5> ddq) {
  int order = 5; // polynomial order

  Matrix<double, Dynamic, 1> time = Matrix<double, Dynamic, 1>::LinSpaced(t,0,1);
  VectorNd a = qs;
  VectorNd b = qf;

  VectorNd c = VectorNd::Zeoros(N);
  VectorNd d = 10 * (qf - qs) - (6 * dqs + 4 * dqf);
  VectorNd e = -15 * (qf - qs) + (8 * dqs + 7 * dqf);
  VectorNd f = 6 * (qf - qs) - (3 * dqf + 3 * dqs);

  MatrixXd times(t.size(), order + 1);
  for (int i = 0; i <= order; i++) {
    times.col(i) = t.array().pow(i);
  }

  MatrixXd coefs(N, order + 1);
  coefs << a, b, c, d, e, f;
  q = times * coefs.transpose();

  coefs << b, VectorNd::Zeoros(N), 3*d, 4*e, 5*f, VectorNd::Zeoros(N);
  dq = times * coefs.transpose();
  dq = dq * v_max / dq.maxCoeff();

  coefs << VectorNd::Zeoros(N), 6*d, 12*e, 20*f, VectorNd::Zeoros(N), VectorNd::Zeoros(N);
  ddq = times * coefs.transpose();

}

s TrajectroyGenerator::get_arc_for(double t, double v) {
  double phi = v * t / (M_PI * Rc);
  double x = Rc * cos(theta_s + phi);
  double y = Rc * sin(theta_s + phi);
  double alpha = alpha_0 + phi;
  s vecs;
  vecs.x = x;
  vecs.y = y;
  vecs.z = 0;
  vecs.alpha = alpha;
  return vecs;
}