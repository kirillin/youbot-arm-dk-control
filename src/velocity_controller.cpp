#include <velocity_controller.h>


VelocityController::VelocityController(Kinematics _ks): ks(_ks) {
  ROS_INFO("Velocity controller initialisation.");
  arm_position_pub = nh.advertise<brics_actuator::JointPositions>("arm_controller/position_command", 1); /*arm_1/arm_controller/position_command*/
  arm_velocity_pub = nh.advertise<brics_actuator::JointVelocities>("arm_controller/velocity_command", 1);
  //gripper_pub = nh.advertise<brics_actuator::JointPositions>("arm_1/gripper_controller/position_command", 1);
  arm_js_sub = nh.subscribe("joint_states", 10, &VelocityController::js_callback, this);
  joy_sub = nh.subscribe("joy", 10, &VelocityController::joy_callback, this);

  out.open ("error_jacobi.txt");

  /* init vectors s_d and ds_d*/
  ds_d << 0.0, 0.0, -0.01, 0,0,0;
//  s_d <<  0.23795853,  0.15675547,  0.13219284, -3.122599784479932, -0.5165184041578356, 0.31116222223650225;

  s_d <<  0.20795853,  0.20675547,  0.0219284, -3.122599784479932, -0.5165184041578356, 0.31116222223650225;

    //    xyz: [ 0.23795853  0.08675547  0.13219284] qtn: [ 0.95473937  0.15220461  0.25087902 -0.04864365]
//    rpy: (-3.122599784479932, -0.5165184041578356, 0.31116222223650225)
  dq0 << 0, 0, 0, 0, 0;

  /* coefs for controller */
  K << 1, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0,
       0, 0, 1, 0, 0, 0,
       0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 1;
}

VelocityController::~VelocityController(){
  out.close();
  arm_position_pub.shutdown();
  arm_velocity_pub.shutdown();
  arm_js_sub.shutdown();
  joy_sub.shutdown();
}

/*!
 *  Simple velocity controller.
 *
 *  `q` is measurements positions of joints (from `sensor_msgs::JointState& msg`).
 *  `dq = J^{+}(q) * (K e + ds_d)`
 *  `e = s_d - s`
 *  `s = FK(q)`
  */
void VelocityController::js_callback(const sensor_msgs::JointState &msg){

  brics_actuator::JointVelocities arm_velocities;
  vector<brics_actuator::JointValue> point;

  /* feedback measurements'q' */
  Matrix<double, 1, NUMBER_OF_JOINTS> q;
  for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
    q(i) = msg.position[i];
  }

  /* feedback cartesian vector 's'*/
  Vector6d s = ks.forward(q);
  /* error*/
  Vector6d error = -0.5 * K * (s_d - s);

  out << error.transpose() << endl;

  clog << error.transpose() << " " << msg.header.stamp <<endl;

  Vector6d ds = -1 * ds_d;//error;// + ds_d;

  /* pseudo inverse Jacobi matrix compute */
  Matrix<double, N, 6> piJ;
  ks.get_pinv_jacobi_num(q, piJ);


//  Matrix<double, 6, N> J;
//  ks.get_jacobi(q, J);
//  JacobiSVD<MatrixXf> svd(J, ComputeThinU | ComputeThinV);
//  Matrix<double, N, 6> iJ = svd.matrixV() * svd.singularValues().asDiagonal() * svd.matrixU().transpose();
//  VectorNd dq = iJ * ds; /* To manipulator pass and vvjjjuhh */

    VectorNd dq = piJ * ds; /* To manipulator pass and vvjjjuhh */


    std::stringstream joint_name;
  point.resize(NUMBER_OF_JOINTS);
  for (int i = 0; i < NUMBER_OF_JOINTS /*+2*/; i++) {
    if (i < 5) { /* link's joints */
      joint_name.str("");
      joint_name << "arm_joint_" << (i + 1);

      point[i].joint_uri = joint_name.str();
      point[i].value = dq(i);
      point[i].unit = boost::units::to_string(boost::units::si::radian_per_second);
    } else {  /* gripper joints */
      if (i == 5) {
        point[i].joint_uri = "gripper_finger_joint_l";
      } else {
        point[i].joint_uri = "gripper_finger_joint_r";
      }
      point[i].value = 0;
      point[i].unit = boost::units::to_string(boost::units::si::meter_per_second);
    }
  };
  arm_velocities.velocities = point;
  arm_velocity_pub.publish(arm_velocities);
}


/*TESTS*/
void VelocityController::js_callback1(const sensor_msgs::JointState &msg){
  ROS_INFO("js");

  brics_actuator::JointPositions arm_positions;
  vector<brics_actuator::JointValue> point;


  /* feedback measurements'q' */
  Matrix<double, 1, NUMBER_OF_JOINTS> q;
  ConfigurationsOfManipulator qs = ks.inverse(s_d);
  if (qs.solves(0) == true) {
    q = qs.qs.row(0);
  }
//  /* feedback cartesian vector 's'*/
//  Vector6d s = ks.forward(q);

//  /* error*/
//  Vector6d error = K * (s_d - s);
//  Vector6d ds = error + ds_d;
//  clog << ds << endl;
//  /* pseudo inverse Jacobi matrix compute */
//  Matrix<double, N, 6> piJ;
//  ks.get_pinv_jacobi_num(q, piJ);
//
//  VectorNd dq = piJ * ds; /* To manipulator pass and vvjjjuhh */

  std::stringstream joint_name;
  point.resize(NUMBER_OF_JOINTS);
  for (int i = 0; i < NUMBER_OF_JOINTS /*+2*/; i++) {
    if(i < 5) { /* link's joints */
      joint_name.str("");
      joint_name << "arm_joint_" << (i + 1);

      point[i].joint_uri = joint_name.str();
      point[i].value = q(i);
      point[i].unit = boost::units::to_string(boost::units::si::radian_per_second);
    } else {  /* gripper joints */
      if (i == 5) {
        point[i].joint_uri = "gripper_finger_joint_l";
      } else {
        point[i].joint_uri = "gripper_finger_joint_r";
      }
      point[i].value = 0;
      point[i].unit = boost::units::to_string(boost::units::si::meter_per_second);
    }
  };
  arm_positions.positions = point;
  arm_position_pub.publish(arm_positions);
}

void VelocityController::joy_callback(const sensor_msgs::Joy &msg){
  double x = msg.axes[0];
  double y = msg.axes[1];
  double z = msg.axes[3];
  Vector6d s;
  double k = 0.01;
  s(0) = s_d(0) + k*x;
  s(1) = s_d(1) + k*y;
  s(2) = s_d(2) + k*z;
  s.tail(3) << 3.1415, 0, 0;
  ConfigurationsOfManipulator cm = ks.inverse(s);
  if (cm.solves(0) == true) {
    s_d = s;
  }
  // clog << s_d << endl;
}


void VelocityController::work(){
  ros::Rate R(200);
  while(nh.ok()) {
    ros::spinOnce();
    R.sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "velocity_controller_node");

  VectorNd v1; v1 << 0.033, 0.155, 0.135, 0.0, 0.0;
  VectorNd v2; v2 << M_PI/2.0, 0.0, 0.0, M_PI/2.0, 0.0;
  VectorNd v3; v3 << 0.147, 0.0, 0.0, 0.0, 0.218;
  VectorNd v4; v4 << M_PI*169.0/180.0, M_PI*65.0/180.0+M_PI/2, -M_PI*146.0/180.0, M_PI*102.5/180.0+M_PI/2, M_PI*167.5/180.0;
  Kinematics ks(v1, v2, v3, v4);

  VelocityController vc(ks);
  vc.work();

  return 0;
}
