#include <deque>
#include <eigen3/Eigen/Dense>

#include "../include/common_lib.h"
#include "../include/sophus/se3.hpp"
#include "sophus/so3.hpp"

// class ESKF_Base {
//  public:
//   virtual void Init() = 0;
//   virtual ~ESKF_Base() = default;
// };

struct ESKFParams {
  bool en_debug;
  std::string time_str;
  std::string save_path;
  double init_heading;

  vector<double> init_gyro_bias;
  vector<double> init_accel_bias;
  vector<double> init_mag_bias;
  vector<double> posi_cov_init;
  vector<double> velo_cov_init;
  vector<double> ori_cov_init;
  vector<double> gyro_cov_init;
  vector<double> accel_cov_init;
  vector<double> mag_cov_init;
  vector<double> g_cov_init;
  vector<double> vel_mot_noise;
  vector<double> rot_mot_noise;
  vector<double> gyro_mot_noise;
  vector<double> accel_mot_noise;
  vector<double> mag_mot_noise;
  vector<double> posi_meas_noise;
  vector<double> vel_meas_noise;
  vector<double> ori_meas_noise;
};

class ESKF {
 public:
  ESKF() {};
  ~ESKF() {};

  bool Init(sensor_msgs::Imu::Ptr &curr_imu_data, GPSGroup &curr_gps_);
  bool predict(const sensor_msgs::Imu::Ptr &curr_imu_data);
  bool correct(const GPSGroup &curr_gps_data);
  void get_pose(V3D &pos, Eigen::Quaterniond &quat, double &stamp_) const;
  void get_vel(V3D &vel);

 private:
  void set_Q();
  void set_R();
  void set_P();
  bool update_odom_estimation(double dt);
  bool update_errror_state(double t, const Eigen::Vector3d &accel,
                           const Eigen::Vector3d &gyro);
  void eliminate_error();
  void reset_state();

 private:
  // pos, vel, ori, gyro_bias, accel_bias, gravity_bias
  static const unsigned int DIM_STATE_ = 21;
  static const unsigned int DIM_STATE_NOISE = 21;
  // 3D position, 3D velocity, 3D magnetic field
  static const unsigned int DIM_MEASUREMENT = 9;
  static const unsigned int DIM_MEASUREMENT_NOISE = 9;

  static const unsigned int INDEX_STATE_POSI = 0;
  static const unsigned int INDEX_STATE_VEL = 3;
  static const unsigned int INDEX_STATE_ORI = 6;
  static const unsigned int INDEX_STATE_GYRO_BIAS = 9;
  static const unsigned int INDEX_STATE_ACC_BIAS = 12;
  static const unsigned int INDEX_STATE_MAG_BIAS = 15;
  static const unsigned int INDEX_STATE_G_BIAS = 18;

  static const unsigned int INDEX_MEASUREMENT_POSI = 0;
  static const unsigned int INDEX_MEASUREMENT_VEL = 3;
  static const unsigned int INDEX_MEASUREMENT_ORI = 6;

  typedef typename Eigen::Matrix<double, DIM_STATE_, 1> TypeVectorX;
  typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, 1> TypeVectorY;
  typedef typename Eigen::Matrix<double, DIM_STATE_, DIM_STATE_> TypeMatrixF;
  typedef typename Eigen::Matrix<double, DIM_STATE_, DIM_STATE_NOISE>
      TypeMatrixB;
  typedef typename Eigen::Matrix<double, DIM_STATE_NOISE, DIM_STATE_NOISE>
      TypeMatrixQ;
  typedef typename Eigen::Matrix<double, DIM_STATE_, DIM_STATE_> TypeMatrixP;
  typedef typename Eigen::Matrix<double, DIM_STATE_, DIM_MEASUREMENT>
      TypeMatrixK;
  typedef typename Eigen::Matrix<double, DIM_MEASUREMENT_NOISE,
                                 DIM_MEASUREMENT_NOISE>
      TypeMatrixC;
  typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, DIM_STATE_>
      TypeMatrixG;
  typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, DIM_MEASUREMENT>
      TypeMatrixR;

  TypeVectorX X_;
  TypeVectorY Y_;
  TypeMatrixF F_;
  TypeMatrixB B_;
  TypeMatrixQ Q_;
  TypeMatrixP P_;
  TypeMatrixK K_;
  TypeMatrixC C_;
  TypeMatrixG G_;
  TypeMatrixC R_;

  TypeMatrixF Ft_;

  Eigen::Vector3d init_velocity_;
  Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
  Eigen::Isometry3d init_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose_ = Eigen::Isometry3d::Identity();

  Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d mag_bias_ = Eigen::Vector3d::Zero();

  Eigen::Vector3d g_;  // gravity
  Eigen::Vector3d w_;  // 地球自传角速度

  GPSGroup curr_gps_data_;

  std::deque<sensor_msgs::Imu::Ptr> imu_data_buff_;
  // std::deque<IMUData> imu_data_buff_;

  double curr_timestamp = 0.0;

 public:
  ESKFParams eskf_params;
  void get_FGY(TypeMatrixF &F, TypeMatrixG &G, TypeVectorY &Y);
  Eigen::Matrix3d BuildSkewMatrix(const Eigen::Vector3d &vec);
  int count_debug = 0;
  double kDegree2Radian = M_PI / 180.0;

  double earth_rotation_speed = 7.272205216e-05;
  double gravity = 9.79484197226504;
  bool flg_eskf_init = false;
};

Eigen::Matrix3d ESKF::BuildSkewMatrix(const Eigen::Vector3d &vec) {
  Eigen::Matrix3d matrix;
  matrix << 0.0, -vec[2], vec[1], vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0;

  return matrix;
}

// pior covariance
void ESKF::set_P() {
  P_.setZero();
  P_.diagonal() << eskf_params.posi_cov_init[0], eskf_params.posi_cov_init[1],
      eskf_params.posi_cov_init[2], eskf_params.velo_cov_init[0],
      eskf_params.velo_cov_init[1], eskf_params.velo_cov_init[2],
      eskf_params.ori_cov_init[0], eskf_params.ori_cov_init[1],
      eskf_params.ori_cov_init[2], eskf_params.gyro_cov_init[0],
      eskf_params.gyro_cov_init[1], eskf_params.gyro_cov_init[2],
      eskf_params.accel_cov_init[0], eskf_params.accel_cov_init[1],
      eskf_params.accel_cov_init[2], eskf_params.mag_cov_init[0],
      eskf_params.mag_cov_init[1], eskf_params.mag_cov_init[2],
      eskf_params.g_cov_init[0], eskf_params.g_cov_init[1],
      eskf_params.g_cov_init[2];
}
// process noise
void ESKF::set_Q() {
  Q_.setZero();
  Q_.diagonal() << 0, 0, 0, eskf_params.vel_mot_noise[0],
      eskf_params.vel_mot_noise[1], eskf_params.vel_mot_noise[2],
      eskf_params.rot_mot_noise[0], eskf_params.rot_mot_noise[1],
      eskf_params.rot_mot_noise[2], eskf_params.gyro_mot_noise[0],
      eskf_params.gyro_mot_noise[1], eskf_params.gyro_mot_noise[2],
      eskf_params.accel_mot_noise[0], eskf_params.accel_mot_noise[1],
      eskf_params.accel_mot_noise[2], eskf_params.mag_mot_noise[0],
      eskf_params.mag_mot_noise[1], eskf_params.mag_mot_noise[2], 0, 0, 0;
}
// measurement noise
void ESKF::set_R() {
  R_.setZero();
  R_.diagonal() << eskf_params.posi_meas_noise[0],
      eskf_params.posi_meas_noise[1], eskf_params.posi_meas_noise[2],
      eskf_params.vel_meas_noise[0], eskf_params.vel_meas_noise[1],
      eskf_params.vel_meas_noise[2], eskf_params.ori_meas_noise[0],
      eskf_params.ori_meas_noise[1], eskf_params.ori_meas_noise[2];
}

bool ESKF::Init(sensor_msgs::Imu::Ptr &curr_imu_data, GPSGroup &curr_gps_) {
  g_ = Eigen::Vector3d(0.0, 0.0, -gravity);
  accel_bias_ = Eigen::Vector3d(eskf_params.init_accel_bias[0],
                                eskf_params.init_accel_bias[1],
                                eskf_params.init_accel_bias[2]);
  gyro_bias_ = Eigen::Vector3d(eskf_params.init_gyro_bias[0],
                               eskf_params.init_gyro_bias[1],
                               eskf_params.init_gyro_bias[2]);
  mag_bias_ = Eigen::Vector3d(eskf_params.init_mag_bias[0],
                              eskf_params.init_mag_bias[1],
                              eskf_params.init_mag_bias[2]);
  set_P();
  set_Q();
  set_R();

  X_.setZero();
  F_.setZero();
  C_.setIdentity();
  G_.block<3, 3>(INDEX_MEASUREMENT_POSI, INDEX_MEASUREMENT_POSI) =
      Eigen::Matrix3d::Identity();
  G_.block<3, 3>(INDEX_MEASUREMENT_VEL, INDEX_MEASUREMENT_VEL) =
      Eigen::Matrix3d::Identity();
  G_.block<3, 3>(INDEX_MEASUREMENT_ORI, INDEX_MEASUREMENT_ORI) =
      Eigen::Matrix3d::Identity();

  Eigen::Quaterniond q_init =
      Eigen::AngleAxisd(eskf_params.init_heading, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
  pose_.matrix().block<3, 3>(0, 0) = q_init.toRotationMatrix();
  pose_.matrix().block<3, 1>(0, 3) = curr_gps_.UTM;
  velocity_ = curr_gps_.velocity;

  imu_data_buff_.clear();
  imu_data_buff_.push_back(curr_imu_data);

  flg_eskf_init = true;

  return true;
}

void ESKF::get_FGY(TypeMatrixF &F, TypeMatrixG &G, TypeVectorY &Y) {
  F = Ft_;
  G = G_;
  Y = Y_;
}
// 观测方程：Y = G * x + R
bool ESKF::correct(const GPSGroup &curr_gps_data) {
  curr_gps_data_ = curr_gps_data;
  Eigen::Vector3d curr_gps_enu, curr_gps_vel, curr_mag, curr_mag_ned;
  curr_gps_enu << curr_gps_data_.UTM[0], curr_gps_data_.UTM[1],
      curr_gps_data_.UTM[2];
  curr_gps_vel = curr_gps_data_.velocity;
  curr_mag = curr_gps_data_.mageto;
  curr_mag_ned = curr_gps_data_.mag_ned;

  // measurement error，Y = gps_mesure - imu_predict, 依赖GPS提供初值
  Y_.block<3, 1>(INDEX_MEASUREMENT_POSI, 0) =
      curr_gps_enu - pose_.translation();
  Y_.block<3, 1>(INDEX_MEASUREMENT_VEL, 0) = curr_gps_vel - velocity_;
  //   Y_.block<3, 1>(INDEX_MEASUREMENT_VEL, 0) = Eigen::Vector3d(0, 0, 0);

  auto scale_ =
      (curr_mag - mag_bias_) *
      (curr_mag_ned.transpose() / (curr_mag_ned.transpose() * curr_mag_ned));
  Sophus::SO3d SO3_R(pose_.rotation().matrix().transpose() * scale_);
  Eigen::Vector3d log_mag = SO3_R.log();
  if ((std::abs(log_mag[2]) - 3.1415926) < 1e-2) {
    log_mag[2] = 0.0;
  }
  if ((std::abs(log_mag[1]) - 3.1415926) < 1e-2) {
    log_mag[1] = 0.0;
  }
  if ((std::abs(log_mag[0]) - 3.1415926) < 1e-2) {
    log_mag[0] = 0.0;
  }
  Y_.block<3, 1>(INDEX_MEASUREMENT_ORI, 0) =
      log_mag - X_.block<3, 1>(INDEX_STATE_ORI, 0);
  //   Y_.block<3, 1>(INDEX_MEASUREMENT_ORI, 0) = SO3_R.log();
  //   Y_.block<3, 1>(INDEX_MEASUREMENT_ORI, 0) = Eigen::Vector3d(0, 0, 0);

  K_ = P_ * G_.transpose() *
       (G_ * P_ * G_.transpose() + C_ * R_ * C_.transpose()).inverse();
  P_ = (TypeMatrixP::Identity() - K_ * G_) * P_;
  X_ = K_ * Y_;  // TODO

  // ======= DEBUG: is residual covergence? ========= //
  if (eskf_params.en_debug) {
    double gt_heading = atan2(curr_mag[1], curr_mag[0]);
    Eigen::Quaterniond q_heading =
        Eigen::AngleAxisd(gt_heading, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());

    std::string write_path_1 =
        eskf_params.save_path + "gt_" + eskf_params.time_str + ".txt";
    std::ofstream outfile_1;
    outfile_1.open(write_path_1, std::ofstream::app);
    outfile_1 << setprecision(19) << curr_timestamp << " "
              << curr_gps_data_.UTM[0] << " " << curr_gps_data_.UTM[1] << " "
              << curr_gps_data_.UTM[2] << " " << q_heading.x() << " "
              << q_heading.y() << " " << q_heading.z() << " " << q_heading.w()
              << std::endl;
    outfile_1.close();

    std::string write_path_2 =
        eskf_params.save_path + "vel_enu_" + eskf_params.time_str + ".txt";
    std::ofstream outfile_2;
    outfile_2.open(write_path_2, std::ofstream::app);
    outfile_2 << setprecision(19) << curr_timestamp << " " << curr_gps_vel[0]
              << " " << curr_gps_vel[1] << " " << curr_gps_vel[2] << " " << 0
              << " " << 0 << " " << 0 << " " << 1 << std::endl;
    outfile_2.close();

    std::string write_path_3 =
        eskf_params.save_path + "theta_" + eskf_params.time_str + ".txt";
    std::ofstream outfile_3;
    outfile_3.open(write_path_3, std::ofstream::app);
    outfile_3 << setprecision(19) << curr_timestamp << " " << SO3_R.log()[0]
              << " " << SO3_R.log()[1] << " " << SO3_R.log()[2] << " " << 0
              << " " << 0 << " " << 0 << " " << 1 << std::endl;
    outfile_3.close();
    std::string write_path_4 =
        eskf_params.save_path + "residual_" + eskf_params.time_str + ".txt";
    std::ofstream outfile_4;
    outfile_4.open(write_path_4, std::ofstream::app);
    outfile_4 << setprecision(19) << curr_timestamp << " " << Y_[0] << " "
              << Y_[1] << " " << Y_[2] << " " << Y_[3] << " " << Y_[4] << " "
              << Y_[5] << " " << Y_[6] << " " << Y_[7] << " " << Y_[8]
              << std::endl;
  }

  eliminate_error();
  reset_state();

  return true;
}

bool ESKF::predict(const sensor_msgs::Imu::Ptr &curr_imu_data) {
  sensor_msgs::Imu::Ptr curr_imu_(new sensor_msgs::Imu(*curr_imu_data));
  Eigen::Vector3d curr_meas_acc, curr_meas_gyro;
  curr_meas_acc << curr_imu_data->linear_acceleration.x,
      curr_imu_data->linear_acceleration.y,
      curr_imu_data->linear_acceleration.z;
  curr_meas_gyro << curr_imu_data->angular_velocity.x,
      curr_imu_data->angular_velocity.y, curr_imu_data->angular_velocity.z;

  imu_data_buff_.push_back(curr_imu_data);
  curr_timestamp = curr_imu_data->header.stamp.toSec();
  double delta_t = curr_imu_data->header.stamp.toSec() -
                   imu_data_buff_.front()->header.stamp.toSec();
  update_odom_estimation(delta_t);

  update_errror_state(delta_t, curr_meas_acc, curr_meas_gyro);

  imu_data_buff_.pop_front();

  return true;
}

bool ESKF::update_errror_state(double dt, const Eigen::Vector3d &accel,
                               const Eigen::Vector3d &gyro) {
  F_ = TypeMatrixF::Identity();
  F_.block<3, 3>(INDEX_STATE_POSI, INDEX_STATE_VEL) =
      Eigen::Matrix3d::Identity() * dt;
  F_.block<3, 3>(INDEX_STATE_VEL, INDEX_STATE_ORI) =
      -pose_.rotation().matrix() * Sophus::SO3d::hat(accel - accel_bias_) * dt;
  F_.block<3, 3>(INDEX_STATE_VEL, INDEX_STATE_ACC_BIAS) =
      -pose_.rotation().matrix() * dt;
  F_.block<3, 3>(INDEX_STATE_VEL, INDEX_STATE_G_BIAS) =
      Eigen::Matrix3d::Identity() * dt;
  F_.block<3, 3>(INDEX_STATE_ORI, INDEX_STATE_ORI) =
      Sophus::SO3d::exp(-(gyro - gyro_bias_) * dt).matrix();
  F_.block<3, 3>(INDEX_STATE_ORI, INDEX_STATE_GYRO_BIAS) =
      -Eigen::Matrix3d::Identity() * dt;

  // TODO:
  //   B_.block<3, 3>(INDEX_STATE_VEL, 3) = pose_.rotation().matrix() * dt;
  //   B_.block<3, 3>(INDEX_STATE_ORI, 0) = -pose_.rotation().matrix() * dt;
  // update state：xk = Fk-1 * xk-1
  X_ = F_ * X_;
  // The covariance matrix of the error state
  //   P_ = F_ * P_ * F_.transpose() + B_ * Q_ * B_.transpose();
  P_ = F_ * P_ * F_.transpose() + Q_;

  // ========= DEBUG: is covergence? ========= //
  //   std::string write_path_1 =
  //   "/home/mint/ws_fusion_uwb/src/inno_ligo/data/res/P_gyro.txt";
  //   std::ofstream outfile_1;
  //   outfile_1.open(write_path_1, std::ofstream::app);
  //   outfile_1 << setprecision(19) << curr_timestamp << " " <<
  //   P_(INDEX_STATE_GYRO_BIAS, INDEX_STATE_GYRO_BIAS) << " "
  //             << P_(INDEX_STATE_GYRO_BIAS + 1, INDEX_STATE_GYRO_BIAS + 1) <<
  //             " "
  //             << P_(INDEX_STATE_GYRO_BIAS + 2, INDEX_STATE_GYRO_BIAS + 2) <<
  //             " " << 0 << " " << 0 << " " << 0 << " "
  //             << 1
  //             << std::endl;
  //   outfile_1.close();

  //   std::string write_path_2 =
  //   "/home/mint/ws_fusion_uwb/src/inno_ligo/data/res/P_acc.txt";
  //   std::ofstream outfile_2;
  //   outfile_2.open(write_path_2, std::ofstream::app);
  //   outfile_2 << setprecision(19) << curr_timestamp << " " <<
  //   P_(INDEX_STATE_ACC_BIAS, INDEX_STATE_ACC_BIAS) << " "
  //             << P_(INDEX_STATE_ACC_BIAS + 1, INDEX_STATE_ACC_BIAS + 1) << "
  //             "
  //             << P_(INDEX_STATE_ACC_BIAS + 2, INDEX_STATE_ACC_BIAS + 2) << "
  //             " << 0 << " " << 0 << " " << 0 << " " <<
  //             1
  //             << std::endl;
  //   outfile_2.close();

  return true;
}

bool ESKF::update_odom_estimation(double dt) {
  sensor_msgs::Imu::Ptr curr_imu_data(
      new sensor_msgs::Imu(*imu_data_buff_.at(1)));
  sensor_msgs::Imu::Ptr last_imu_data(
      new sensor_msgs::Imu(*imu_data_buff_.at(0)));
  Eigen::Vector3d curr_accel, curr_gyro;
  curr_accel << curr_imu_data->linear_acceleration.x,
      curr_imu_data->linear_acceleration.y,
      curr_imu_data->linear_acceleration.z;
  curr_gyro << curr_imu_data->angular_velocity.x,
      curr_imu_data->angular_velocity.y, curr_imu_data->angular_velocity.z;
  // new_p = p_ + v_ * dt + 0.5 * (R_ * (imu.acce_ - ba_)) * dt * dt +
  //                0.5 * g_ * dt * dt;
  pose_.translation() +=
      velocity_ * dt +
      0.5 * dt * dt * (pose_.rotation().matrix() * (curr_accel - accel_bias_)) +
      0.5 * g_ * dt * dt;
  // new_v = v_ + R_ * (imu.acce_ - ba_) * dt + g_ * dt;
  velocity_ +=
      pose_.rotation().matrix() * (curr_accel - accel_bias_) * dt + g_ * dt;
  // new_R = R_ * SO3::exp((imu.gyro_ - bg_) * dt);
  pose_.rotate(Sophus::SO3d::exp((curr_gyro - gyro_bias_) * dt).matrix());

  if (eskf_params.en_debug) {
    auto acc_ = (curr_accel - accel_bias_) + g_;
    std::string write_path_1 =
        eskf_params.save_path + "acc_norot_" + eskf_params.time_str + ".txt";
    std::ofstream outfile_1;
    outfile_1.open(write_path_1, std::ofstream::app);
    outfile_1 << setprecision(19) << curr_timestamp << " " << acc_(0) << " "
              << acc_(1) << " " << acc_(2) << " " << 0 << " " << 0 << " " << 0
              << " " << 1 << std::endl;
    outfile_1.close();

    auto acc_1_ = pose_.rotation().matrix() * (curr_accel - accel_bias_) + g_;
    Eigen::Quaterniond tmp_q(pose_.rotation().matrix());
    std::string write_path_2 =
        eskf_params.save_path + "acc_debug_" + eskf_params.time_str + ".txt";
    std::ofstream outfile_2;
    outfile_2.open(write_path_2, std::ofstream::app);
    outfile_2 << setprecision(19) << curr_timestamp << " " << acc_1_(0) << " "
              << acc_1_(1) << " " << acc_1_(2) << " " << tmp_q.x() << " "
              << tmp_q.y() << " " << tmp_q.z() << " " << tmp_q.w() << std::endl;
    outfile_2.close();
  }
  auto acc_ = curr_accel + g_;
  std::string write_path_3 = eskf_params.save_path + "acc_nobias_debug_" +
                             eskf_params.time_str + ".txt";
  std::ofstream outfile_3;
  outfile_3.open(write_path_3, std::ofstream::app);
  outfile_3 << setprecision(19) << curr_timestamp << " " << acc_(0) << " "
            << acc_(1) << " " << acc_(2) << " " << 0 << " " << 0 << " " << 0
            << " " << 1 << std::endl;
  outfile_3.close();

  return true;
}

void ESKF::reset_state() {
  // project covariance matrix
  TypeMatrixF J = TypeMatrixF::Identity();
  J.block<3, 3>(INDEX_STATE_ORI, INDEX_STATE_ORI) =
      Eigen::Matrix3d::Identity() -
      0.5 * Sophus::SO3d::hat(X_.block<3, 1>(INDEX_STATE_ORI, 0));
  P_ = J * P_ * J.transpose();

  X_.setZero();
}

void ESKF::eliminate_error() {
  pose_.translation() += X_.block<3, 1>(INDEX_STATE_POSI, 0);

  velocity_ += X_.block<3, 1>(INDEX_STATE_VEL, 0);
  Eigen::Matrix3d C_nn =
      Sophus::SO3d::exp(X_.block<3, 1>(INDEX_STATE_ORI, 0)).matrix();
  //   pose_.rotation().matrix() = pose_.rotation().matrix() * C_nn;
  pose_.rotate(C_nn);

  //   Eigen::Quaterniond q_tmp(C_nn);
  //   Eigen::Vector3d eulerAngle = q_tmp.matrix().eulerAngles(0, 1, 2);
  //   if ((std::abs(eulerAngle.x() * 180 / M_PI) < 10 ||
  //        std::abs(std::abs(eulerAngle.x() * 180 / M_PI) - 180) < 10) &&
  //       (std::abs(eulerAngle.y() * 180 / M_PI) < 10 ||
  //        std::abs(std::abs(eulerAngle.y() * 180 / M_PI) - 180) < 10) &&
  //       (std::abs(eulerAngle.z() * 180 / M_PI < 10) ||
  //        std::abs(std::abs(eulerAngle.z() * 180 / M_PI) - 180) < 10)) {
  //     pose_.rotate(C_nn);
  //   } else {
  //     ROS_WARN("Error: eulerAngle: %f %f %f", eulerAngle.x() * 180 / M_PI,
  //              eulerAngle.y() * 180 / M_PI, eulerAngle.z() * 180 / M_PI);
  //   }

  // TODO
  gyro_bias_ += X_.block<3, 1>(INDEX_STATE_GYRO_BIAS, 0);
  accel_bias_ += X_.block<3, 1>(INDEX_STATE_ACC_BIAS, 0);
  mag_bias_ += X_.block<3, 1>(INDEX_STATE_MAG_BIAS, 0);
  g_ += X_.block<3, 1>(INDEX_STATE_G_BIAS, 0);

  if (eskf_params.en_debug) {
    std::string write_path_1 =
        eskf_params.save_path + "error_state_" + eskf_params.time_str + ".txt";
    std::ofstream outfile_1;
    outfile_1.open(write_path_1, std::ofstream::app);
    outfile_1 << setprecision(19) << curr_timestamp << " " << X_[0] << " "
              << X_[1] << " " << X_[2] << " " << X_[3] << " " << X_[4] << " "
              << X_[5] << " " << X_[6] << " " << X_[7] << " " << X_[8] << " "
              << X_[9] << " " << X_[10] << " " << X_[11] << " " << X_[12] << " "
              << X_[13] << " " << X_[14] << " " << X_[15] << " " << X_[16]
              << " " << X_[17] << " " << X_[18] << " " << X_[19] << " "
              << X_[20] << std::endl;
    outfile_1.close();

    Eigen::Vector3d rota_ = pose_.rotation().matrix().eulerAngles(2, 1, 0);
    std::string write_path_2 =
        eskf_params.save_path + "state_" + eskf_params.time_str + ".txt";
    std::ofstream outfile_2;
    outfile_2.open(write_path_2, std::ofstream::app);
    outfile_2 << setprecision(19) << curr_timestamp << " "
              << pose_.translation()[0] << " " << pose_.translation()[1] << " "
              << pose_.translation()[2] << " " << velocity_[0] << " "
              << velocity_[1] << " " << velocity_[2] << " " << rota_[0] << " "
              << rota_[1] << " " << rota_[2] << " " << gyro_bias_[0] << " "
              << gyro_bias_[1] << " " << gyro_bias_[2] << " " << accel_bias_[0]
              << " " << accel_bias_[1] << " " << accel_bias_[2] << " "
              << mag_bias_[0] << " " << mag_bias_[1] << " " << mag_bias_[2]
              << " " << g_[0] << " " << g_[1] << " " << g_[2] << std::endl;
  }
}

void ESKF::get_pose(V3D &pos, Eigen::Quaterniond &quat,
                    double &timestamp_) const {
  pos << pose_(0, 3), pose_(1, 3), pose_(2, 3);
  quat = Eigen::Quaterniond(pose_.rotation().matrix());
  // pose_;
  timestamp_ = curr_timestamp;
}

void ESKF::get_vel(V3D &vel) { vel = velocity_; }
