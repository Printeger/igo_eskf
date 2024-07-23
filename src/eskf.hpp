#include <deque>
#include <eigen3/Eigen/Dense>

#include "../include/common_lib.h"
#include "../include/sophus/se3.hpp"

class ESKF {
 public:
  ESKF(){};
  ~ESKF(){};

  bool Init(sensor_msgs::Imu::Ptr &curr_imu_data, const vector<double> &cov_prior_pos,
            const vector<double> &cov_prior_vel, const vector<double> &cov_prior_ori,
            const vector<double> &cov_prior_epsilon, const vector<double> &cov_prior_delta,
            const vector<double> &cov_meas_pos, const vector<double> &cov_proc_gyro,
            const vector<double> &cov_proc_acc);
  bool Init(sensor_msgs::Imu::Ptr &curr_imu_data, const vector<double> &cov_prior_pos,
            const vector<double> &cov_prior_vel, const vector<double> &cov_prior_ori,
            const vector<double> &cov_prior_epsilon, const vector<double> &cov_prior_delta,
            const vector<double> &cov_proc_gyro, const vector<double> &cov_proc_acc, const vector<double> &cov_meas_pos,
            const vector<double> &cov_meas_vel);
  /*!
   * 滤波器的预测，对应卡尔曼滤波器的前两个公式
   * @param curr_imu_data
   * @return
   */
  bool Predict(const sensor_msgs::Imu::Ptr &curr_imu_data);
  /*!
   * 滤波器的矫正，对应卡尔曼滤波器的后三个公式
   * @param curr_gps_data
   * @return
   */
  bool Correct(const GPSGroup &curr_gps_data);

  // Eigen::Matrix4d GetPose() const;
  void GetPose(V3D &pos, Eigen::Quaterniond &quat, double &stamp_) const;

  Eigen::Vector3d GetVelocity() { return velocity_; }

 private:
   void SetCovarianceQ(const vector<double> &gyro_noise_cov, const vector<double> &accel_noise_cov);
   void SetCovarianceR(const vector<double> &posi_noise_cov);
   void SetCovarianceR(const vector<double> &posi_meas_cov, const vector<double> &velo_meas_cov);
   void SetCovarianceP(const vector<double> &posi_noise, const vector<double> &velo_noise,
                       const vector<double> &ori_noise, const vector<double> &gyro_noise,
                       const vector<double> &accel_noise);

   /*!
    * 通过IMU计算位姿和速度
    * @return
    */
   bool UpdateOdomEstimation();

   bool UpdateErrorState(double t, const Eigen::Vector3d &accel);

   bool ComputeAngularDelta(Eigen::Vector3d &angular_delta);

   /*!
    * 计算地球转动给导航系带来的变换
    * @param R_nm_nm_1
    * @return
    */
   bool ComputeEarthTranform(Eigen::Matrix3d &R_nm_nm_1);

   /*!
    * 通过IMU计算当前姿态
    * @param angular_delta
    * @param R_nm_nm_1
    * @param curr_R
    * @param last_R
    * @return
    */
   bool ComputeOrientation(const Eigen::Vector3d &angular_delta, const Eigen::Matrix3d R_nm_nm_1,
                           Eigen::Matrix3d &curr_R, Eigen::Matrix3d &last_R);

   bool ComputeVelocity(Eigen::Vector3d &curr_vel, Eigen::Vector3d &last_vel, const Eigen::Matrix3d &curr_R,
                        const Eigen::Matrix3d last_R);

   Eigen::Vector3d GetUnbiasAccel(const Eigen::Vector3d &accel);

   /*!
    * 通过imu计算当前位移
    * @param curr_vel
    * @param last_vel
    * @return
    */
   bool ComputePosition(const Eigen::Vector3d &curr_vel, const Eigen::Vector3d &last_vel);

   /*!
    * 对误差进行滤波之后，需要在实际算出来的轨迹中，消除这部分误差
    */
   void EliminateError();

   /*!
    * 每次矫正之后，需要重置状态变量X
    */
   void ResetState();

 private:
  static const unsigned int DIM_STATE_ = 15;
  static const unsigned int DIM_STATE_NOISE = 6;
  static const unsigned int DIM_MEASUREMENT = 6;
  static const unsigned int DIM_MEASUREMENT_NOISE = 6;

  static const unsigned int INDEX_STATE_POSI = 0;
  static const unsigned int INDEX_STATE_VEL = 3;
  static const unsigned int INDEX_STATE_ORI = 6;
  static const unsigned int INDEX_STATE_GYRO_BIAS = 9;
  static const unsigned int INDEX_STATE_ACC_BIAS = 12;
  static const unsigned int INDEX_MEASUREMENT_POSI = 0;
  static const unsigned int INDEX_MEASUREMENT_VEL = 3;

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
  Eigen::Matrix4d init_pose_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();

  Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias_ = Eigen::Vector3d::Zero();

  Eigen::Vector3d g_;  // 重力加速度
  Eigen::Vector3d w_;  // 地球自传角速度

  GPSGroup curr_gps_data_;
  double L_ = 0.0; // 纬度
  std::deque<sensor_msgs::Imu::Ptr> imu_data_buff_;
  // std::deque<IMUData> imu_data_buff_;
  double curr_timestamp = 0.0;

public:
  void GetFGY(TypeMatrixF &F, TypeMatrixG &G, TypeVectorY &Y);
  int count_debug = 0;
  double earth_rotation_speed = 7.272205216e-05;
  double gravity = 9.79484197226504;
  bool flg_eskf_init = false;
};

constexpr double kDegree2Radian = M_PI / 180.0;

Eigen::Matrix3d BuildSkewMatrix(const Eigen::Vector3d &vec) {
  Eigen::Matrix3d matrix;
  matrix << 0.0, -vec[2], vec[1], vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0;

  return matrix;
}

// P: 先验的协方差  R：测量的协方差   Q：过程的协方差
void ESKF::SetCovarianceQ(const vector<double> &gyro_noise_cov, const vector<double> &accel_noise_cov)
{
    //   Q_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * gyro_noise * gyro_noise;
    // Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * accel_noise * accel_noise;
    Q_.setZero();
    Q_(0, 0) = gyro_noise_cov[0] * gyro_noise_cov[0];
    Q_(1, 1) = gyro_noise_cov[1] * gyro_noise_cov[1];
    Q_(2, 2) = gyro_noise_cov[2] * gyro_noise_cov[2];
    Q_(3, 3) = accel_noise_cov[0] * accel_noise_cov[0];
    Q_(4, 4) = accel_noise_cov[1] * accel_noise_cov[1];
    Q_(5, 5) = accel_noise_cov[2] * accel_noise_cov[2];
}

void ESKF::SetCovarianceR(const vector<double> &posi_noise_cov)
{
    // R_ = Eigen::Matrix3d::Identity() * posi_noise * posi_noise;
    R_.setZero();
    R_(0, 0) = posi_noise_cov[0] * posi_noise_cov[0];
    R_(1, 1) = posi_noise_cov[1] * posi_noise_cov[1];
    R_(2, 2) = posi_noise_cov[2] * posi_noise_cov[2];
}

void ESKF::SetCovarianceR(const vector<double> &posi_meas_cov, const vector<double> &velo_meas_cov)
{
    R_.setZero();
    R_(0, 0) = posi_meas_cov[0] * posi_meas_cov[0];
    R_(1, 1) = posi_meas_cov[1] * posi_meas_cov[1];
    R_(2, 2) = posi_meas_cov[2] * posi_meas_cov[2];
    R_(3, 3) = velo_meas_cov[0] * velo_meas_cov[0];
    R_(4, 4) = velo_meas_cov[1] * velo_meas_cov[1];
    R_(5, 5) = velo_meas_cov[2] * velo_meas_cov[2];
}

void ESKF::SetCovarianceP(const vector<double> &posi_noise, const vector<double> &velo_noise,
                          const vector<double> &ori_noise, const vector<double> &gyro_noise,
                          const vector<double> &accel_noise)
{
    P_.setZero();
    for (int i = 0; i < 3; i++)
    {
        P_.block<3, 3>(INDEX_STATE_POSI + i, INDEX_STATE_POSI + i) = Eigen::Matrix3d::Identity() * posi_noise[i];
    }
    for (int i = 0; i < 3; i++)
    {
        P_.block<3, 3>(INDEX_STATE_VEL + i, INDEX_STATE_VEL + i) = Eigen::Matrix3d::Identity() * velo_noise[i];
    }
    for (int i = 0; i < 3; i++)
    {
        P_.block<3, 3>(INDEX_STATE_ORI + i, INDEX_STATE_ORI + i) = Eigen::Matrix3d::Identity() * ori_noise[i];
    }
    for (int i = 0; i < 3; i++)
    {
        P_.block<3, 3>(INDEX_STATE_GYRO_BIAS + i, INDEX_STATE_GYRO_BIAS + i) =
            Eigen::Matrix3d::Identity() * gyro_noise[i];
    }
    for (int i = 0; i < 3; i++)
    {
        P_.block<3, 3>(INDEX_STATE_ACC_BIAS + i, INDEX_STATE_ACC_BIAS + i) =
            Eigen::Matrix3d::Identity() * accel_noise[i];
    }
}

bool ESKF::Init(sensor_msgs::Imu::Ptr &curr_imu_data, const vector<double> &cov_prior_pos,
                const vector<double> &cov_prior_vel, const vector<double> &cov_prior_ori,
                const vector<double> &cov_prior_epsilon, const vector<double> &cov_prior_delta,
                const vector<double> &cov_proc_gyro, const vector<double> &cov_proc_acc,
                const vector<double> &cov_meas_pos)
{
    g_ = Eigen::Vector3d(0.0, 0.0, -gravity);
    // w_ = Eigen::Vector3d(0.0, earth_rotation_speed * cos(L_ * kDegree2Radian),
    //                      earth_rotation_speed * sin(L_ * kDegree2Radian));
    w_ = Eigen::Vector3d(0.0, 0.0, 0.0);

    SetCovarianceP(cov_prior_pos, cov_prior_vel, cov_prior_ori, cov_prior_epsilon, cov_prior_delta);
    SetCovarianceR(cov_meas_pos);
    SetCovarianceQ(cov_proc_gyro, cov_proc_acc);

    X_.setZero();
    F_.setZero();
    C_.setIdentity();
    G_.block<3, 3>(INDEX_MEASUREMENT_POSI, INDEX_MEASUREMENT_POSI) = Eigen::Matrix3d::Identity();
    G_.block<3, 3>(INDEX_MEASUREMENT_VEL, INDEX_MEASUREMENT_VEL) = Eigen::Matrix3d::Identity();
    F_.block<3, 3>(INDEX_STATE_POSI, INDEX_STATE_VEL) = Eigen::Matrix3d::Identity();
    F_.block<3, 3>(INDEX_STATE_ORI, INDEX_STATE_ORI) = BuildSkewMatrix(-w_);
    init_velocity_ << 0, 0, 0;
    velocity_ = init_velocity_;
    Eigen::Quaterniond Q = Eigen::AngleAxisd(90 * kDegree2Radian, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(0 * kDegree2Radian, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(180 * kDegree2Radian, Eigen::Vector3d::UnitX());
    init_pose_.block<3, 3>(0, 0) = Q.toRotationMatrix();
    pose_ = init_pose_;

    imu_data_buff_.clear();
    imu_data_buff_.push_back(curr_imu_data);

    flg_eskf_init = true;

    return true;
}

bool ESKF::Init(sensor_msgs::Imu::Ptr &curr_imu_data, const vector<double> &cov_prior_pos,
                const vector<double> &cov_prior_vel, const vector<double> &cov_prior_ori,
                const vector<double> &cov_prior_epsilon, const vector<double> &cov_prior_delta,
                const vector<double> &cov_proc_gyro, const vector<double> &cov_proc_acc,
                const vector<double> &cov_meas_pos, const vector<double> &cov_meas_vel)
{
    g_ = Eigen::Vector3d(0.0, 0.0, -gravity);
    // w_ = Eigen::Vector3d(0.0, earth_rotation_speed * cos(L_ * kDegree2Radian),
    //                      earth_rotation_speed * sin(L_ * kDegree2Radian));
    w_ = Eigen::Vector3d(0.0, 0.0, 0.0);

    SetCovarianceP(cov_prior_pos, cov_prior_vel, cov_prior_ori, cov_prior_epsilon, cov_prior_delta);
    SetCovarianceR(cov_meas_pos, cov_meas_vel);
    SetCovarianceQ(cov_proc_gyro, cov_proc_acc);

    X_.setZero();
    F_.setZero();
    C_.setIdentity();
    G_.block<3, 3>(INDEX_MEASUREMENT_POSI, INDEX_MEASUREMENT_POSI) = Eigen::Matrix3d::Identity();
    G_.block<3, 3>(INDEX_MEASUREMENT_VEL, INDEX_MEASUREMENT_VEL) = Eigen::Matrix3d::Identity();
    F_.block<3, 3>(INDEX_STATE_POSI, INDEX_STATE_VEL) = Eigen::Matrix3d::Identity();
    F_.block<3, 3>(INDEX_STATE_ORI, INDEX_STATE_ORI) = BuildSkewMatrix(-w_);
    init_velocity_ << 0, 0, 0;
    velocity_ = init_velocity_;
    Eigen::Quaterniond Q = Eigen::AngleAxisd(90 * kDegree2Radian, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(0 * kDegree2Radian, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(180 * kDegree2Radian, Eigen::Vector3d::UnitX());
    init_pose_.block<3, 3>(0, 0) = Q.toRotationMatrix();
    pose_ = init_pose_;

    imu_data_buff_.clear();
    imu_data_buff_.push_back(curr_imu_data);

    flg_eskf_init = true;

    return true;
}

void ESKF::GetFGY(TypeMatrixF &F, TypeMatrixG &G, TypeVectorY &Y) {
  F = Ft_;
  G = G_;
  Y = Y_;
}
// 观测方程：Y = G * x + R
bool ESKF::Correct(const GPSGroup &curr_gps_data)
{
    curr_timestamp = curr_gps_data.timestamp;
    curr_gps_data_ = curr_gps_data;
    Eigen::Vector3d curr_gps_ned, curr_gps_vel;
    curr_gps_ned << curr_gps_data_.UTM[0], curr_gps_data_.UTM[1], curr_gps_data_.UTM[2];
    curr_gps_vel << curr_gps_data_.velocity[0], curr_gps_data_.velocity[1], curr_gps_data_.velocity[2];
    // 测量误差，Y = gps_mesure - imu_predict, 依赖GPS提供初值
    // Eigen::Matrix<double, DIM_MEASUREMENT, 1> TypeVectorY ;
    // Y_.setZero();
    ROS_INFO("curr_gps_ned: %f, %f, %f", curr_gps_ned[0], curr_gps_ned[1], curr_gps_ned[2]);
    ROS_INFO("curr_gps_vel: %f, %f, %f", curr_gps_vel[0], curr_gps_vel[1], curr_gps_vel[2]);
    ROS_INFO("velocity_: %f, %f, %f", velocity_[0], velocity_[1], velocity_[2]);
    Eigen::Vector3d tmp_ = Eigen::Vector3d::Zero();
    Y_.block<3, 1>(0, 0) = pose_.block<3, 1>(0, 3) - curr_gps_ned;
    Y_.block<3, 1>(3, 0) = velocity_ - curr_gps_vel;

    // Kalman 增益
    K_ = P_ * G_.transpose() * (G_ * P_ * G_.transpose() + C_ * R_ * C_.transpose()).inverse();
    // 后验的协方差矩阵
    P_ = (TypeMatrixP::Identity() - K_ * G_) * P_;
    // 后验估计
    X_ = X_ + K_ * (Y_ - G_ * X_);

    // ======= DEBUG: is residual covergence? ========= //
    std::string write_path_1 = "/home/mint/ws_fusion_uwb/src/igo_eskf/data/res/residual_vel.txt";
    std::ofstream outfile_1;
    outfile_1.open(write_path_1, std::ofstream::app);
    outfile_1 << setprecision(19) << curr_timestamp << " " << Y_(3) << " " << Y_(4) << " " << Y_(5) << " " << 0 << " "
              << 0 << " " << 0 << " " << 1 << std::endl;
    outfile_1.close();
    std::string write_path_2 = "/home/mint/ws_fusion_uwb/src/igo_eskf/data/res/residual_posi.txt";
    std::ofstream outfile_2;
    outfile_2.open(write_path_2, std::ofstream::app);
    outfile_2 << setprecision(19) << curr_timestamp << " " << Y_(0) << " " << Y_(1) << " " << Y_(2) << " " << 0 << " "
              << 0 << " " << 0 << " " << 1 << std::endl;
    outfile_2.close();
    // std::cout << "X_: " << X_ << std::endl;
    // 迭代完后，将最终得到的误差重新带回到预测的位置、速度、方向中消除掉这个误差
    EliminateError();
    // 因为是对状态的误差进行矫正，所以ESKF每次迭代完成后要将状态量X重设为零
    ResetState();

    return true;
}

bool ESKF::Predict(const sensor_msgs::Imu::Ptr &curr_imu_data) {
  imu_data_buff_.push_back(curr_imu_data);

  UpdateOdomEstimation();

  double delta_t = curr_imu_data->header.stamp.toSec() -
                   imu_data_buff_.front()->header.stamp.toSec();

  Eigen::Vector3d curr_meas_acc;
  curr_meas_acc << curr_imu_data->linear_acceleration.x,
      curr_imu_data->linear_acceleration.y,
      curr_imu_data->linear_acceleration.z;

  Eigen::Vector3d curr_accel = pose_.block<3, 3>(0, 0) * curr_meas_acc;
  UpdateErrorState(delta_t, curr_accel);

  imu_data_buff_.pop_front();

  return true;
}

bool ESKF::UpdateErrorState(double t, const Eigen::Vector3d &accel) {
  Eigen::Matrix3d F_23 = BuildSkewMatrix(accel);

  F_.block<3, 3>(INDEX_STATE_VEL, INDEX_STATE_ORI) = F_23;
  F_.block<3, 3>(INDEX_STATE_VEL, INDEX_STATE_ACC_BIAS) = pose_.block<3, 3>(0, 0);
  F_.block<3, 3>(INDEX_STATE_ORI, INDEX_STATE_GYRO_BIAS) = -pose_.block<3, 3>(0, 0);
  B_.block<3, 3>(INDEX_STATE_VEL, 3) = pose_.block<3, 3>(0, 0);
  B_.block<3, 3>(INDEX_STATE_ORI, 0) = -pose_.block<3, 3>(0, 0);

  // 状态方程：x^ = F(t) * x + B(t) * w
  //  状态方程一阶线形展开
  TypeMatrixF Fk = TypeMatrixF::Identity() + F_ * t;
  TypeMatrixB Bk = B_ * t;

  Ft_ = F_ * t;
  // 更新状态：xk = Fk-1 * xk-1
  X_ = Fk * X_;
  // 误差状态的协方差矩阵
  P_ = Fk * P_ * Fk.transpose() + Bk * Q_ * Bk.transpose();

  // ========= DEBUG: is covergence? ========= //
  //   std::string write_path_1 = "/home/mint/ws_fusion_uwb/src/inno_ligo/data/res/P_gyro.txt";
  //   std::ofstream outfile_1;
  //   outfile_1.open(write_path_1, std::ofstream::app);
  //   outfile_1 << setprecision(19) << curr_timestamp << " " << P_(INDEX_STATE_GYRO_BIAS, INDEX_STATE_GYRO_BIAS) << " "
  //             << P_(INDEX_STATE_GYRO_BIAS + 1, INDEX_STATE_GYRO_BIAS + 1) << " "
  //             << P_(INDEX_STATE_GYRO_BIAS + 2, INDEX_STATE_GYRO_BIAS + 2) << " " << 0 << " " << 0 << " " << 0 << " "
  //             << 1
  //             << std::endl;
  //   outfile_1.close();

  //   std::string write_path_2 = "/home/mint/ws_fusion_uwb/src/inno_ligo/data/res/P_acc.txt";
  //   std::ofstream outfile_2;
  //   outfile_2.open(write_path_2, std::ofstream::app);
  //   outfile_2 << setprecision(19) << curr_timestamp << " " << P_(INDEX_STATE_ACC_BIAS, INDEX_STATE_ACC_BIAS) << " "
  //             << P_(INDEX_STATE_ACC_BIAS + 1, INDEX_STATE_ACC_BIAS + 1) << " "
  //             << P_(INDEX_STATE_ACC_BIAS + 2, INDEX_STATE_ACC_BIAS + 2) << " " << 0 << " " << 0 << " " << 0 << " " <<
  //             1
  //             << std::endl;
  //   outfile_2.close();

  return true;
}

bool ESKF::UpdateOdomEstimation() {
  Eigen::Vector3d angular_delta;
  ComputeAngularDelta(angular_delta);

  Eigen::Matrix3d R_nm_nm_1 = Eigen::Matrix3d::Identity();
  //   ComputeEarthTranform(R_nm_nm_1);

  Eigen::Matrix3d curr_R, last_R;
  ComputeOrientation(angular_delta, R_nm_nm_1, curr_R, last_R);

  Eigen::Vector3d curr_vel, last_vel;
  ComputeVelocity(curr_vel, last_vel, curr_R, last_R);

  ComputePosition(curr_vel, last_vel);

  return true;
}
// 根据IMU计算两帧IMU时间中，走过角度
bool ESKF::ComputeAngularDelta(Eigen::Vector3d &angular_delta) {
  sensor_msgs::Imu::Ptr curr_imu_data(
      new sensor_msgs::Imu(*imu_data_buff_.at(1)));
  sensor_msgs::Imu::Ptr last_imu_data(
      new sensor_msgs::Imu(*imu_data_buff_.at(0)));

  double delta_t =
      curr_imu_data->header.stamp.toSec() - last_imu_data->header.stamp.toSec();

  if (delta_t <= 0) {
    return false;
  }
  Eigen::Vector3d curr_angular_vel;
  curr_angular_vel << curr_imu_data->angular_velocity.x,
      curr_imu_data->angular_velocity.y, curr_imu_data->angular_velocity.z;

  Eigen::Vector3d last_angular_vel;
  last_angular_vel << last_imu_data->angular_velocity.x,
      last_imu_data->angular_velocity.y, last_imu_data->angular_velocity.z;

  Eigen::Vector3d curr_unbias_angular_vel = curr_angular_vel;
  Eigen::Vector3d last_unbias_angular_vel = last_angular_vel;

  angular_delta =
      0.5 * (curr_unbias_angular_vel + last_unbias_angular_vel) * delta_t;

  // std::string write_path =
  //     "/home/xng/catkin_ws/src/inno_lio/data/res/angvel_avr_valid.txt";
  // std::ofstream outfile;
  // outfile.open(write_path, std::ofstream::app);
  // outfile << count_debug << " " << angular_delta[0] << " " <<
  // angular_delta[1]
  //         << " " << angular_delta[2] << " " << 0 << " " << 0 << " " << 0 << "
  //         "
  //         << delta_t << std::endl;
  // outfile.close();
  // count_debug++;

  return true;
}
// 根据GPS测速计算两帧IMU时间中，走过角度的观测值
bool ESKF::ComputeEarthTranform(Eigen::Matrix3d &R_nm_nm_1) {
  sensor_msgs::Imu::Ptr curr_imu_data(
      new sensor_msgs::Imu(*imu_data_buff_.at(1)));
  sensor_msgs::Imu::Ptr last_imu_data(
      new sensor_msgs::Imu(*imu_data_buff_.at(0)));

  double delta_t =
      curr_imu_data->header.stamp.toSec() - last_imu_data->header.stamp.toSec();

  constexpr double rm = 6353346.18315;  // 极半径
  constexpr double rn = 6384140.52699;  // 赤道半径
  // TODO  根据GPS测速计算观测的角速度
  //   Eigen::Vector3d w_en_n(-velocity_[1] / (rm + curr_gps_data_.UTM[2]), velocity_[0] / (rn + curr_gps_data_.UTM[2]),
  //                          velocity_[0] / (rn + curr_gps_data_.UTM[2]) *
  //                              std::tan(curr_gps_data_.LLA[0] * kDegree2Radian));
  // *0215
  Eigen::Vector3d w_en_n(velocity_[1] / (rm + curr_gps_data_.UTM[2]),
                         velocity_[0] / (rn + curr_gps_data_.UTM[2]) * std::tan(curr_gps_data_.LLA[0] * kDegree2Radian),
                         velocity_[0] / (rn + curr_gps_data_.UTM[2]));
  // 考虑地球自传角速度w_
  Eigen::Vector3d w_in_n = w_en_n + w_;
  // Eigen::Vector3d w_in_n = w_;

  auto angular = delta_t * w_in_n;

  Eigen::AngleAxisd angle_axisd(angular.norm(), angular.normalized());

  R_nm_nm_1 = angle_axisd.toRotationMatrix().transpose();

  return true;
}
// predict pose = R_GPS.inv() * R_IMU_measured * R_consider_lastframe_imu_meas
bool ESKF::ComputeOrientation(const Eigen::Vector3d &angular_delta,
                              const Eigen::Matrix3d R_nm_nm_1,
                              Eigen::Matrix3d &curr_R,
                              Eigen::Matrix3d &last_R) {
  Eigen::AngleAxisd angle_axisd(angular_delta.norm(),
                                angular_delta.normalized());
  last_R = pose_.block<3, 3>(0, 0);
  // TODO===TODO===TODO===TODO===TODO===
  curr_R = R_nm_nm_1 * pose_.block<3, 3>(0, 0) * angle_axisd.toRotationMatrix();
  // curr_R = pose_.block<3, 3>(0, 0) * angle_axisd.toRotationMatrix();

  pose_.block<3, 3>(0, 0) = curr_R;

  Eigen::Vector3d ea_ = curr_R.eulerAngles(2, 1, 0);
  // ===============DEBUG===============
  // std::string write_path_1 =
  //     "/home/xng/catkin_ws/src/inno_ligo/data/res/debug_currR_.txt";
  // std::ofstream outfile_1;
  // outfile_1.open(write_path_1, std::ofstream::app);
  // outfile_1 << " " << ea_[0] << " " << ea_[1] << " " << ea_[2] << " " << 0
  //           << " " << 0 << " " << 0 << " " << 1 << std::endl;
  // outfile_1.close();

  return true;
}
// vel_consider_lastframe
bool ESKF::ComputeVelocity(Eigen::Vector3d &curr_vel, Eigen::Vector3d &last_vel,
                           const Eigen::Matrix3d &curr_R,
                           const Eigen::Matrix3d last_R) {
  sensor_msgs::Imu::Ptr curr_imu_data(
      new sensor_msgs::Imu(*imu_data_buff_.at(1)));
  sensor_msgs::Imu::Ptr last_imu_data(
      new sensor_msgs::Imu(*imu_data_buff_.at(0)));
  double delta_t =
      curr_imu_data->header.stamp.toSec() - last_imu_data->header.stamp.toSec();
  if (delta_t <= 0) {
    return false;
  }

  Eigen::Vector3d curr_accel;
  curr_accel << curr_imu_data->linear_acceleration.x,
      curr_imu_data->linear_acceleration.y,
      curr_imu_data->linear_acceleration.z;
  Eigen::Vector3d curr_unbias_accel = GetUnbiasAccel(curr_R * curr_accel);

  Eigen::Vector3d last_accel;
  last_accel << last_imu_data->linear_acceleration.x,
      last_imu_data->linear_acceleration.y,
      last_imu_data->linear_acceleration.z;
  Eigen::Vector3d last_unbias_accel = GetUnbiasAccel(last_R * last_accel);

  last_vel = velocity_;

  velocity_ += delta_t * 0.5 * (curr_unbias_accel + last_unbias_accel);
  curr_vel = velocity_;

  // ===============DEBUG===============
  // std::string write_path_1 =
  //     "/home/xng/catkin_ws/src/inno_ligo/data/res/acc_curr.txt";
  // std::ofstream outfile_1;
  // outfile_1.open(write_path_1, std::ofstream::app);
  // outfile_1 << " " << curr_unbias_accel[0] << " " << curr_unbias_accel[1] <<
  // " "
  //           << curr_unbias_accel[2] << " " << 0 << " " << 0 << " " << 0 << "
  //           "
  //           << 1 << std::endl;
  // outfile_1.close();
  // debug_num++;

  return true;
}

Eigen::Vector3d ESKF::GetUnbiasAccel(const Eigen::Vector3d &accel) {
    // TODO
    return accel - accel_bias_ + g_;
    // return accel + g_;
}
// position consider last frame
bool ESKF::ComputePosition(const Eigen::Vector3d &curr_vel,
                           const Eigen::Vector3d &last_vel) {
  double delta_t = imu_data_buff_.at(1)->header.stamp.toSec() -
                   imu_data_buff_.at(0)->header.stamp.toSec();

  pose_.block<3, 1>(0, 3) += 0.5 * delta_t * (curr_vel + last_vel);

  return true;
}

void ESKF::ResetState() { X_.setZero(); }

void ESKF::EliminateError() {
  pose_.block<3, 1>(0, 3) =
      pose_.block<3, 1>(0, 3) - X_.block<3, 1>(INDEX_STATE_POSI, 0);

  velocity_ = velocity_ - X_.block<3, 1>(INDEX_STATE_VEL, 0);
  Eigen::Matrix3d C_nn =
      Sophus::SO3d::exp(X_.block<3, 1>(INDEX_STATE_ORI, 0)).matrix();
  pose_.block<3, 3>(0, 0) = C_nn * pose_.block<3, 3>(0, 0);

  gyro_bias_ = gyro_bias_ - X_.block<3, 1>(INDEX_STATE_GYRO_BIAS, 0);
  accel_bias_ = accel_bias_ - X_.block<3, 1>(INDEX_STATE_ACC_BIAS, 0);
}

// Eigen::Matrix4d ESKF::GetPose() const { return pose_; }

void ESKF::GetPose(V3D &pos, Eigen::Quaterniond &quat, double &timestamp_) const
{
    pos << pose_(0, 3), pose_(1, 3), pose_(2, 3);
    quat = Eigen::Quaterniond(pose_.block<3, 3>(0, 0));
    // pose_;
    timestamp_ = curr_timestamp;
}