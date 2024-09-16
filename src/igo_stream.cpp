// DONE: 1. 修正所有传感器时间戳，设置其为收到时的系统时间；
// DONE: 2. IMU mag初始化，统计offset
// DONE: 3. Transform Velocity to body axis
// DONE: 4. Add velocity to ESKF   [TODO: NEED TO VERIFY!!]
// TODO: 5. 使用基类，实现ESKF和ESKF_VEL
#include <Python.h>
// #include <ikd-Tree/ikd_Tree.h>
#include <math.h>
#include <omp.h>
#include <ros/ros.h>
#include <so3_math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <unistd.h>

// msgs
// #include <common_msgs/LinktrackNodeframe2.h>
#include <geometry_msgs/Vector3.h>
#include <gnss_comm/GnssPVTSolnMsg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Core>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

#include "../include/wmm/GeomagnetismLibrary.hpp"
#include "eskf.hpp"
#include "eskf_vel.hpp"
#include "gnss_process.hpp"

#define INIT_TIME (0.1)
#define GPS_COV (0.1)

// DEBUG
std::string time_str;
int meas_num = 0;
double gravity = 9.79484197226504;
double rad2degree = 57.29577951308232;
double degree2rad = M_PI / 180.0;

std::mutex mtx_buffer;
condition_variable sig_buffer;

ofstream fout_pre, fout_out, fout_dbg;
double solve_time = 0, solve_const_H_time = 0;
double res_mean_last = 0.05, total_residual = 0.0;  // 设置残差平均值，残差总和
double init_mag_heading = 0.0;
double curr_heading_vel = 0.0;
double curr_heading_angle = 0.0;
double last_mag_heading = 0.0;
double last_mag_stamp = 0.0;
double curr_mag_stamp = 0.0;
double mag_bias = 0.0;
double curr_imu_stamp = 0.0;
double last_imu_stamp = 0.0;

int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0,
    laserCloudValidNum = 0, count_ = 0, imu_slide_window_size = 10;
double last_timestamp_imu = -1.0, last_timestamp_gps = 0, first_gps_time = 0.0,
       last_timestamp_uwb, gps_curr_time = 0.0, imu_filter_n_sigma = 3,
       init_duration = 0.0, init_init_stamp = 1e19, zero_gyro_threshold = 0.0;

deque<double> time_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_window_buffer;
deque<GPSGroup> gps_buffer;
deque<GPSGroup> uwb_buffer;
deque<geometry_msgs::PoseStamped> vicon_buffer;
deque<sensor_msgs::MagneticField> mavros_mag_buffer;

bool flg_first_gps = true, path_en = true, flg_EKF_inited, en_vicon = false,
     en_debug = false, is_mag_heading_init = false, en_time_sync = false,
     en_sensor_init = false, en_rtk_vel = false, is_sensor_init = false,
     is_imu_recv = false, is_1st_pose = true;
std::string imu_topic, gps_topic, uwb_topic, vicon_topic, mag_topic,
    file_save_path, pose_topic, odom_topic, path_topic, wmm_cof_path;
bool TRANSAXIS = true;

Eigen::Isometry3d init_pose = Eigen::Isometry3d::Identity();

// GPS with respect to IMU
V3D GPS_T_wrt_IMU(Zero3d);
M3D GPS_R_wrt_IMU(Eye3d);
V3D euler_cur;
V3D res_pos;
V3D res_vel;
V3D sum_acc(Zero3d), sum_gyr(Zero3d);
V3D mean_acc(Zero3d), mean_gyr(Zero3d);
V3D acc_offset(Zero3d);
V3D gyr_offset(Zero3d);
V3D std_mag(Zero3d);
int cnt_imu = 0;
Eigen::Quaterniond res_quat(1, 0, 0, 0);

// shared_ptr<ImuProcess> imu_proc(new ImuProcess());
shared_ptr<GPSProcess> gps_proc(new GPSProcess());
shared_ptr<WMMProcess> mag_proc(new WMMProcess());
shared_ptr<GPSProcess> uwb_proc(new GPSProcess());

ESKF eskf_proc;

// OUTPUT
nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;

bool sensor_init(deque<sensor_msgs::Imu::ConstPtr> &imu_buffer,
                 deque<sensor_msgs::MagneticField> &mag_buffer_,
                 V3D &acc_offset_, V3D &gyr_offset_, V3D &std_mag_,
                 double &init_heading) {
  if (imu_buffer.size() < 100 || mag_buffer_.size() < 100) {
    return false;
  }
  // calculate imu & mag offset
  V3D sum_acc(Zero3d), sum_gyr(Zero3d), sum_mag(Zero3d), mean_mag(Zero3d);
  double sum_heading = 0.0;
  for (const auto &imu : imu_buffer) {
    sum_acc += V3D(imu->linear_acceleration.x, imu->linear_acceleration.y,
                   imu->linear_acceleration.z - gravity);
    sum_gyr += V3D(imu->angular_velocity.x, imu->angular_velocity.y,
                   imu->angular_velocity.z);
  }
  for (const auto &mag : mag_buffer_) {
    sum_mag +=
        V3D(mag.magnetic_field.x, mag.magnetic_field.y, mag.magnetic_field.z);
  }
  acc_offset_ = sum_acc / imu_buffer.size();
  gyr_offset_ = sum_gyr / imu_buffer.size();
  mean_mag = sum_mag / mag_buffer_.size();
  // calculate mag std
  // V3D sum_unbias_mag(Zero3d);
  // for (const auto &mag : mag_buffer_) {
  //   auto tmp_mag = mag;
  //   tmp_mag.magnetic_field.x -= mean_mag[0];
  //   tmp_mag.magnetic_field.y -= mean_mag[1];
  //   tmp_mag.magnetic_field.z -= mean_mag[2];
  //   sum_unbias_mag += V3D(tmp_mag.magnetic_field.x *
  //   tmp_mag.magnetic_field.x,
  //                         tmp_mag.magnetic_field.y *
  //                         tmp_mag.magnetic_field.y, tmp_mag.magnetic_field.z
  //                         * tmp_mag.magnetic_field.z);
  // }
  // std_mag_ << std::sqrt(sum_unbias_mag[0] / mag_buffer_.size()),
  //     std::sqrt(sum_unbias_mag[1] / mag_buffer_.size()),
  //     std::sqrt(sum_unbias_mag[2] / mag_buffer_.size());
  // // remove ouliers
  // // std::cout << "std_mag_:" << std_mag_ << std::endl;
  // double filtered_mag_x = 0.0, filtered_mag_y = 0.0;
  // int cnt_x = 0, cnt_y = 0;
  // for (const auto &mag : mag_buffer_) {
  //   if (std::abs(mag.magnetic_field.x) < imu_filter_n_sigma * std_mag_[0]) {
  //     filtered_mag_x += mag.magnetic_field.x;
  //     cnt_x++;
  //   }
  //   if (std::abs(mag.magnetic_field.y) < imu_filter_n_sigma * std_mag_[1]) {
  //     filtered_mag_y += mag.magnetic_field.y;
  //     cnt_y++;
  //   }
  // }
  // if (cnt_x == 0 || cnt_y == 0) {
  //   ROS_ERROR("Magnetic field data is too noisy!");
  //   return false;
  // }
  // filtered_mag_x /= cnt_x;
  // filtered_mag_y /= cnt_y;
  init_heading = atan2(mean_mag[1], mean_mag[0]);
  if (init_heading > M_PI) {
    init_heading -= 2 * M_PI;
  }
  if (init_heading < -M_PI) {
    init_heading += 2 * M_PI;
  }
  return true;
}

double get_stamp() {
  auto now = std::chrono::system_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      now.time_since_epoch());
  uint64_t unix_timestamp = duration.count();
  return unix_timestamp / 1e6;
}

void format_imu(const sensor_msgs::Imu::ConstPtr &imu_in,
                sensor_msgs::Imu::Ptr &imu_out) {
  imu_out->header.stamp = imu_in->header.stamp;
  imu_out->header.frame_id = imu_in->header.frame_id;
  // IMU+: x: forward, y: left, z: up -> x: forward, y: right, z: down
  imu_out->linear_acceleration.x =
      imu_in->linear_acceleration.x * gravity - acc_offset[0];
  imu_out->linear_acceleration.y =
      imu_in->linear_acceleration.y * gravity - acc_offset[1];
  imu_out->linear_acceleration.z =
      imu_in->linear_acceleration.z * gravity - acc_offset[2];  // TODO

  // imu_out->linear_acceleration.x =
  //     imu_in->linear_acceleration.x - acc_offset[0];
  // imu_out->linear_acceleration.y =
  //     imu_in->linear_acceleration.y - acc_offset[1];
  // imu_out->linear_acceleration.z =
  //     imu_in->linear_acceleration.z - acc_offset[2];

  // zero gyro update
  if (std::abs(imu_out->linear_acceleration.x) < zero_gyro_threshold &&
      std::abs(imu_out->linear_acceleration.y) < zero_gyro_threshold) {
    imu_out->angular_velocity.x = 0.0;
    imu_out->angular_velocity.y = 0.0;
    imu_out->angular_velocity.z = 0.0;
    // std::cout << "zero gyro update" << std::endl;
  } else {
    // imu_out->angular_velocity.x =
    //     (imu_in->angular_velocity.x - gyr_offset[0]) * M_PI / 180;
    // imu_out->angular_velocity.y =
    //     (imu_in->angular_velocity.y - gyr_offset[1]) * M_PI / 180;
    // imu_out->angular_velocity.z =
    //     (imu_in->angular_velocity.z - gyr_offset[2]) * M_PI / 180;
    imu_out->angular_velocity.x = (imu_in->angular_velocity.x - gyr_offset[0]);
    imu_out->angular_velocity.y = (imu_in->angular_velocity.y - gyr_offset[1]);
    imu_out->angular_velocity.z = (imu_in->angular_velocity.z - gyr_offset[2]);
  }

  if (en_debug) {
    std::string write_path1 = file_save_path + "acc_raw_" + time_str + ".txt";
    std::ofstream outfile1;
    outfile1.open(write_path1, std::ofstream::app);
    outfile1 << setprecision(19) << imu_in->header.stamp.toSec() << " "
             << imu_out->linear_acceleration.x << " "
             << imu_out->linear_acceleration.y << " "
             << imu_out->linear_acceleration.z << " " << 0 << " " << 0 << " "
             << 0 << " " << 1 << std::endl;
    outfile1.close();
    std::string write_path2 = file_save_path + "gyro_raw_" + time_str + ".txt";
    std::ofstream outfile2;
    outfile2.open(write_path2, std::ofstream::app);
    outfile2 << setprecision(19) << imu_in->header.stamp.toSec() << " "
             << imu_in->angular_velocity.x << " " << imu_in->angular_velocity.y
             << " " << imu_in->angular_velocity.z << " " << 0 << " " << 0 << " "
             << 0 << " " << 1 << std::endl;
    outfile2.close();
  }
}

uint64_t convertGpsToUnix(uint32_t gpsWeek, uint32_t gpsTow) {
  // GPS epoch (January 6, 1980 00:00:00 UTC)
  const uint64_t gpsEpoch = 315964800;

  // GPS week duration in seconds
  const uint64_t secondsPerWeek = 604800;

  // Convert GPS week and TOW to seconds since GPS epoch
  uint64_t gpsSeconds = gpsWeek * secondsPerWeek + gpsTow;

  // Calculate Unix timestamp by adding GPS seconds to GPS epoch
  uint64_t unixTimestamp = gpsEpoch + gpsSeconds;

  return unixTimestamp;
}

bool calc_local_mag_field(GPSGroup &gps) {
  MAGtype_MagneticModel *MagneticModel;
  MAGtype_Geoid Geoid;
  MAGtype_Ellipsoid Ellip;
  MAGtype_CoordGeodetic CoordGeodetic;
  MAGtype_Date UserDate;
  MAGtype_GeoMagneticElements GeoMagneticElements;
  MAGtype_CoordSpherical CoordSpherical;

  mag_proc->MAG_SetDefaults(&Ellip, &Geoid);

  mag_proc->MAG_robustReadMagModels(
      const_cast<char *>(wmm_cof_path.c_str()),
      (MAGtype_MagneticModel * (*)[]) & MagneticModel, 1);
  if (MagneticModel == NULL) {
    std::cerr << "Error loading WMM model." << std::endl;
    return false;
  }
  time_t rawtime = time(NULL);
  struct tm *timeinfo = localtime(&rawtime);
  UserDate.Year = timeinfo->tm_year + 1900;
  UserDate.Month = timeinfo->tm_mon + 1;
  UserDate.Day = timeinfo->tm_mday;
  UserDate.DecimalYear =
      UserDate.Year + (UserDate.Month - 1) / 12.0 + (UserDate.Day - 1) / 365.25;

  CoordGeodetic.phi = gps.LLA[0];                   // latitude
  CoordGeodetic.lambda = gps.LLA[1];                // longitude
  CoordGeodetic.HeightAboveEllipsoid = gps.LLA[2];  // altitude
  // 转换地理坐标到球面坐标
  mag_proc->MAG_GeodeticToSpherical(Ellip, CoordGeodetic, &CoordSpherical);

  // 计算地磁场向量
  mag_proc->MAG_Geomag(Ellip, CoordSpherical, CoordGeodetic, MagneticModel,
                       &GeoMagneticElements);
  // 1 （nT） = 0.00001 （Guess）
  // gps.mag_ned[0] = GeoMagneticElements.X * 0.00001;  // nT -> Guess
  // gps.mag_ned[1] = GeoMagneticElements.Y * 0.00001;
  // gps.mag_ned[2] = GeoMagneticElements.Z * 0.00001;
  gps.mag_ned[0] = GeoMagneticElements.X * 0.00001;  // nT -> Guess
  gps.mag_ned[1] = GeoMagneticElements.Y * 0.00001;
  gps.mag_ned[2] = GeoMagneticElements.Z * 0.00001;
  std::cout << "heading: " << atan2(gps.mag_ned[1], gps.mag_ned[0])
            << std::endl;

  mag_proc->MAG_FreeMagneticModelMemory(MagneticModel);
  return true;
}

bool sync_mag_gps(GPSGroup &gps) {
  if (mavros_mag_buffer.empty()) {
    ROS_WARN("mag_buffer is empty");
    return false;
  }
  double gps_time = gps.timestamp;
  double min_duration = 1e19;
  double duration = 0.0;
  // Find the vicon pose with the closest timestamp
  sensor_msgs::MagneticField closest_mag;
  auto iter = mavros_mag_buffer.begin();
  if (!mavros_mag_buffer.empty()) {
    closest_mag = mavros_mag_buffer.front();
    // min_duration = std::abs(input.header.stamp.toSec() -
    // closest_mag.header.stamp.toSec());
    for (auto it = mavros_mag_buffer.begin(); it != mavros_mag_buffer.end();
         ++it) {
      duration = std::abs(gps_time - it->header.stamp.toSec());
      if (duration < min_duration) {
        min_duration = duration;
        closest_mag = *it;
        iter = it;
      }
    }
    mavros_mag_buffer.erase(mavros_mag_buffer.begin(), prev(iter));
  }
  // Origin Mag is NED, trans to ENU
  gps.mageto = V3D(closest_mag.magnetic_field.x, closest_mag.magnetic_field.y,
                   closest_mag.magnetic_field.z);

  if (calc_local_mag_field(gps)) {
    return true;
  } else {
    ROS_ERROR("calc_local_mag_field failed");
    return false;
  }

  return true;
}

sensor_msgs::Imu::Ptr slidingWindowFilter(
    deque<sensor_msgs::Imu::ConstPtr> &imu_buffer_window, const int WINDOW_SIZE,
    const double OUTLIER_THRESHOLD) {
  // Initialize variables for the sliding window and mean values
  V3D mean_acc(Zero3d);
  V3D mean_gyr(Zero3d);
  double mean_stamp = 0.0;
  // Remove the highest and lowest IMU measurements before calculate the mean
  // acceleration and angular velocity over the window

  for (const auto &imu : imu_buffer_window) {
    mean_acc += V3D(imu->linear_acceleration.x, imu->linear_acceleration.y,
                    imu->linear_acceleration.z);
    mean_gyr += V3D(imu->angular_velocity.x, imu->angular_velocity.y,
                    imu->angular_velocity.z);
    mean_stamp += imu->header.stamp.toSec();
  }
  mean_acc /= WINDOW_SIZE;
  mean_gyr /= WINDOW_SIZE;
  mean_stamp /= WINDOW_SIZE;
  // Calculate the standard deviation of acceleration and angular velocity
  V3D std_acc(Zero3d);
  V3D std_gyr(Zero3d);
  for (const auto &imu : imu_buffer_window) {
    const V3D acc_diff =
        V3D(imu->linear_acceleration.x, imu->linear_acceleration.y,
            imu->linear_acceleration.z) -
        mean_acc;
    const V3D gyr_diff = V3D(imu->angular_velocity.x, imu->angular_velocity.y,
                             imu->angular_velocity.z) -
                         mean_gyr;
    std_acc += acc_diff.cwiseProduct(acc_diff);
    std_gyr += gyr_diff.cwiseProduct(gyr_diff);
  }
  std_acc = std_acc.cwiseSqrt() / WINDOW_SIZE;
  std_gyr = std_gyr.cwiseSqrt() / WINDOW_SIZE;
  // Remove outliers from the window buffer
  deque<sensor_msgs::Imu::ConstPtr> filtered_buffer_;
  std::pair<std::deque<sensor_msgs::Imu::ConstPtr>::iterator, double>
      max_acc_diff = std::make_pair(imu_buffer_window.begin(), 0.0);
  std::pair<std::deque<sensor_msgs::Imu::ConstPtr>::iterator, double>
      max_gyr_diff = std::make_pair(imu_buffer_window.begin(), 0.0);
  for (auto it = imu_buffer_window.begin(); it != imu_buffer_window.end();) {
    const auto &imu = *it;
    const V3D acc_diff =
        V3D(imu->linear_acceleration.x, imu->linear_acceleration.y,
            imu->linear_acceleration.z) -
        mean_acc;
    const V3D gyr_diff = V3D(imu->angular_velocity.x, imu->angular_velocity.y,
                             imu->angular_velocity.z) -
                         mean_gyr;
    const double acc_norm = acc_diff.norm();
    const double gyr_norm = gyr_diff.norm();
    if (acc_norm > max_acc_diff.second) {
      max_acc_diff = std::make_pair(it, acc_norm);
    }
    if (gyr_norm > max_gyr_diff.second) {
      max_gyr_diff = std::make_pair(it, gyr_norm);
    }
    // Check if the current IMU measurement is an outlier
    if (acc_norm < OUTLIER_THRESHOLD * std_acc.norm() &&
        gyr_norm < OUTLIER_THRESHOLD * std_gyr.norm()) {
      filtered_buffer_.push_back(imu);
    }
    ++it;
  }

  if (filtered_buffer_.size() == 0) {
    if (max_acc_diff.first == max_gyr_diff.first) {
      imu_buffer_window.erase(max_acc_diff.first);
    } else if (max_acc_diff.first < max_gyr_diff.first) {
      imu_buffer_window.erase(max_gyr_diff.first);
      imu_buffer_window.erase(max_acc_diff.first);
    } else {
      imu_buffer_window.erase(max_acc_diff.first);
      imu_buffer_window.erase(max_gyr_diff.first);
    }
    V3D mean_acc(Zero3d);
    V3D mean_gyr(Zero3d);
    double mean_stamp = 0.0;
    for (const auto &imu : imu_buffer_window) {
      mean_acc += V3D(imu->linear_acceleration.x, imu->linear_acceleration.y,
                      imu->linear_acceleration.z);
      mean_gyr += V3D(imu->angular_velocity.x, imu->angular_velocity.y,
                      imu->angular_velocity.z);
      mean_stamp += imu->header.stamp.toSec();
    }
    mean_acc /= imu_buffer_window.size();
    mean_gyr /= imu_buffer_window.size();
    mean_stamp /= imu_buffer_window.size();
    sensor_msgs::Imu::Ptr imu_out(new sensor_msgs::Imu());
    imu_out->header.stamp = ros::Time().fromSec(mean_stamp);
    imu_out->header.frame_id = imu_buffer_window.back()->header.frame_id;
    imu_out->linear_acceleration.x = mean_acc[0];
    imu_out->linear_acceleration.y = mean_acc[1];
    imu_out->linear_acceleration.z = mean_acc[2];
    imu_out->angular_velocity.x = mean_gyr[0];
    imu_out->angular_velocity.y = mean_gyr[1];
    imu_out->angular_velocity.z = mean_gyr[2];
    // Clear the window buffer for the next window
    imu_buffer_window.clear();

    return imu_out;
  } else {
    // Calculate the mean acceleration and angular velocity after removing
    // outliers
    V3D f_mean_acc = Zero3d;
    V3D f_mean_gyr = Zero3d;
    double f_mean_stamp = 0.0;
    for (const auto &imu : filtered_buffer_) {
      f_mean_acc += V3D(imu->linear_acceleration.x, imu->linear_acceleration.y,
                        imu->linear_acceleration.z);
      f_mean_gyr += V3D(imu->angular_velocity.x, imu->angular_velocity.y,
                        imu->angular_velocity.z);
      f_mean_stamp += imu->header.stamp.toSec();
    }
    f_mean_acc /= filtered_buffer_.size();
    f_mean_gyr /= filtered_buffer_.size();
    f_mean_stamp /= filtered_buffer_.size();
    sensor_msgs::Imu::Ptr imu_out(new sensor_msgs::Imu());
    imu_out->header.stamp = ros::Time().fromSec(f_mean_stamp);
    imu_out->header.frame_id = filtered_buffer_.back()->header.frame_id;
    imu_out->linear_acceleration.x = f_mean_acc[0];
    imu_out->linear_acceleration.y = f_mean_acc[1];
    imu_out->linear_acceleration.z = f_mean_acc[2];
    imu_out->angular_velocity.x = f_mean_gyr[0];
    imu_out->angular_velocity.y = f_mean_gyr[1];
    imu_out->angular_velocity.z = f_mean_gyr[2];
    // Clear the window buffer for the next window
    imu_buffer_window.clear();

    return imu_out;
  }
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) {
  if (!is_imu_recv) {
    init_init_stamp = get_stamp();
    is_imu_recv = true;
  }
  sensor_msgs::Imu::Ptr tmp_msg(new sensor_msgs::Imu(*msg_in));
  if (en_time_sync) {
    double local_stamp = get_stamp();
    tmp_msg->header.stamp = ros::Time().fromSec(local_stamp);
  }
  mtx_buffer.lock();
  double timestamp = tmp_msg->header.stamp.toSec();
  if (timestamp < last_timestamp_imu) {
    ROS_WARN("imu loop back, clear buffer");
    imu_buffer.clear();
  }
  last_timestamp_imu = timestamp;
  if (en_sensor_init && !is_sensor_init) {
    sensor_msgs::Imu::Ptr temp_imu(new sensor_msgs::Imu(*tmp_msg));
    format_imu(tmp_msg, temp_imu);
    imu_buffer.push_back(temp_imu);
  } else {
    sensor_msgs::Imu::Ptr temp_imu(new sensor_msgs::Imu(*tmp_msg));
    format_imu(tmp_msg, temp_imu);

    imu_buffer.push_back(temp_imu);

    // imu_window_buffer.push_back(temp_imu);
    if (imu_window_buffer.size() >= imu_slide_window_size) {
      sensor_msgs::Imu::Ptr imu_filtered = slidingWindowFilter(
          imu_window_buffer, imu_slide_window_size, imu_filter_n_sigma);
      imu_buffer.push_back(imu_filtered);

      if (en_debug) {
        std::string write_path1 =
            file_save_path + "acc_filtered_" + time_str + ".txt";
        std::ofstream outfile1;
        outfile1.open(write_path1, std::ofstream::app);
        outfile1 << setprecision(19) << imu_filtered->header.stamp.toSec()
                 << " " << imu_filtered->linear_acceleration.x << " "
                 << imu_filtered->linear_acceleration.y << " "
                 << imu_filtered->linear_acceleration.z << " " << 0 << " " << 0
                 << " " << 0 << " " << 1 << std::endl;
        outfile1.close();
        std::string write_path2 =
            file_save_path + "gyro_filtered_" + time_str + ".txt";
        std::ofstream outfile2;
        outfile2.open(write_path2, std::ofstream::app);
        outfile2 << setprecision(19) << imu_filtered->header.stamp.toSec()
                 << " " << imu_filtered->angular_velocity.x << " "
                 << imu_filtered->angular_velocity.y << " "
                 << imu_filtered->angular_velocity.z << " " << 0 << " " << 0
                 << " " << 0 << " " << 1 << std::endl;
        outfile2.close();
      }
    }
  }
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

// pixhawk mag: NED, N-0, E-90, Xsens: SWD
void mavros_mag_cbk(const sensor_msgs::MagneticField::ConstPtr &msg) {
  sensor_msgs::MagneticField tmp_msg = *msg;
  mtx_buffer.lock();
  if (en_time_sync) {
    double local_stamp = get_stamp();
    tmp_msg.header.stamp = ros::Time().fromSec(local_stamp);
  }
  tmp_msg.magnetic_field.x = msg->magnetic_field.y;
  tmp_msg.magnetic_field.y = msg->magnetic_field.x;
  tmp_msg.magnetic_field.z = -msg->magnetic_field.z;
  mavros_mag_buffer.push_back(tmp_msg);
  mtx_buffer.unlock();
}

void rtk_cbk(const gnss_comm::GnssPVTSolnMsg::ConstPtr &gps_msg) {
  gnss_comm::GnssPVTSolnMsg::Ptr msg(new gnss_comm::GnssPVTSolnMsg(*gps_msg));
  double timestamp = 0.0;
  if (en_time_sync) {
    timestamp = get_stamp();
    msg->vel_acc = timestamp;
  } else {
    uint64_t recv_stamp = convertGpsToUnix(msg->time.week, msg->time.tow);
    // ROS_INFO("recv_stamp: %ld, msg->time.week: %ld,  msg->time.tow: %ld",
    //          recv_stamp, msg->time.week, msg->time.tow);
    timestamp = static_cast<double>(recv_stamp);  // diff between imu and gps
    msg->vel_acc = timestamp;  // using vel_acc to store timestamp
  }

  if (timestamp < last_timestamp_gps) {
    ROS_WARN("gps loop back, clear buffer");
    gps_buffer.clear();
  }

  if (is_mag_heading_init) {
    // std::cout << "======= init_mag_heading: " << init_mag_heading <<
    // std::endl;
    mtx_buffer.lock();
    GPSGroup temp_utm;
    if (!gps_proc->proj_init) {
      gps_proc->Initialize(gps_msg->longitude, gps_msg->latitude,
                           gps_msg->altitude);
    }
    gps_proc->Process(msg, temp_utm);

    if (!sync_mag_gps(temp_utm)) {
      ROS_WARN("sync mag & gps failed");
    }

    last_timestamp_gps = timestamp;
    time_buffer.push_back(timestamp);
    gps_buffer.push_back(temp_utm);

    if (en_debug) {
      std::string write_path2 = file_save_path + "lla_" + time_str + ".txt";
      std::ofstream outfile2;
      outfile2.open(write_path2, std::ofstream::app);
      outfile2 << setprecision(19) << temp_utm.timestamp << " "
               << temp_utm.LLA[0] << " " << temp_utm.LLA[1] << " "
               << temp_utm.LLA[2] << " " << 0 << " " << 0 << " " << 0 << " "
               << 1 << std::endl;
      outfile2.close();
    }

    mtx_buffer.unlock();
    sig_buffer.notify_all();
  }
}

void gps_cbk(const sensor_msgs::NavSatFix::ConstPtr &gps_msg) {
  ROS_WARN("gps_cbk");
  sensor_msgs::NavSatFix::Ptr msg(new sensor_msgs::NavSatFix(*gps_msg));
  double timestamp = gps_msg->header.stamp.toSec();
  mtx_buffer.lock();

  if (timestamp < last_timestamp_gps) {
    ROS_WARN("gps loop back, clear buffer");
    gps_buffer.clear();
  }
  GPSGroup temp_utm;
  // V4D temp_utm;
  // to utm
  if (!gps_proc->proj_init) {
    gps_proc->Initialize(gps_msg->longitude, gps_msg->latitude,
                         gps_msg->altitude);
  }
  gps_proc->Process(gps_msg, temp_utm);

  last_timestamp_gps = timestamp;
  time_buffer.push_back(timestamp);
  gps_buffer.push_back(temp_utm);
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void vicon_cbk(const geometry_msgs::PoseStamped::ConstPtr &msg_in) {
  double timestamp = 0.0;
  if (en_time_sync) {
    timestamp = get_stamp();
  }
  if (en_debug) {
    std::string write_path2 = file_save_path + "lio_" + time_str + ".txt";
    std::ofstream outfile2;
    outfile2.open(write_path2, std::ofstream::app);
    outfile2 << setprecision(19) << timestamp << " " << msg_in->pose.position.x
             << " " << msg_in->pose.position.y << " " << msg_in->pose.position.z
             << " " << msg_in->pose.orientation.x << " "
             << msg_in->pose.orientation.y << " " << msg_in->pose.orientation.z
             << " " << msg_in->pose.orientation.w << std::endl;
    outfile2.close();
  }
}

// pub pose
template <typename T>
void set_posestamp(T &out) {
  out.pose.position.x = res_pos[0];
  out.pose.position.y = res_pos[1];
  out.pose.position.z = res_pos[2];
  out.pose.orientation.x = res_quat.x();
  out.pose.orientation.y = res_quat.y();
  out.pose.orientation.z = res_quat.z();
  out.pose.orientation.w = res_quat.w();
}

// pub pose
template <typename T>
void set_posestamp(T &out, Eigen::Isometry3d &in_) {
  Eigen::Quaterniond in_quat(in_.rotation().matrix());
  out.pose.position.x = in_.translation()[0];
  out.pose.position.y = in_.translation()[1];
  out.pose.position.z = in_.translation()[2];
  out.pose.orientation.x = in_quat.x();
  out.pose.orientation.y = in_quat.y();
  out.pose.orientation.z = in_quat.z();
  out.pose.orientation.w = in_quat.w();
}

void publish_odometry(const ros::Publisher &pubOdomAftMapped) {
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "body";
  odomAftMapped.header.stamp = ros::Time().fromSec(
      gps_curr_time);  // ros::Time().fromSec(lidar_end_time);
  set_posestamp(odomAftMapped.pose);
  pubOdomAftMapped.publish(odomAftMapped);
  // auto P = kf.get_P();
  // for (int i = 0; i < 6; i++) {
  //   int k = i < 3 ? i + 3 : i - 3;
  //   odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
  //   odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
  //   odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
  //   odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
  //   odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
  //   odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
  // }
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x,
                                  odomAftMapped.pose.pose.position.y,
                                  odomAftMapped.pose.pose.position.z));
  q.setW(odomAftMapped.pose.pose.orientation.w);
  q.setX(odomAftMapped.pose.pose.orientation.x);
  q.setY(odomAftMapped.pose.pose.orientation.y);
  q.setZ(odomAftMapped.pose.pose.orientation.z);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp,
                                        "camera_init", "body"));
}

void publish_path(const ros::Publisher pubPath, ros::Time stamp) {
  set_posestamp(msg_body_pose);
  msg_body_pose.header.stamp = stamp;
  msg_body_pose.header.frame_id = "camera_init";
  /*** if path is too large, the rvis will crash ***/
  static int jjj = 0;
  jjj++;
  if (jjj % 10 == 0) {
    path.poses.push_back(msg_body_pose);
    pubPath.publish(path);
  }
}

Eigen::Isometry3d res2isometry() {
  Eigen::Isometry3d res_isometry = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond curr_quat(res_quat.w(), res_quat.x(), res_quat.y(),
                               res_quat.z());
  res_isometry.rotate(curr_quat.toRotationMatrix());
  res_isometry.pretranslate(
      Eigen::Vector3d(res_pos[0], res_pos[1], res_pos[2]));
  return res_isometry;
}

geometry_msgs::PoseStamped publish_pose(const ros::Publisher pub_pose,
                                        ros::Time stamp) {
  Eigen::Isometry3d curr_T = res2isometry();
  Eigen::Isometry3d rel_T = init_pose.inverse() * curr_T;
  set_posestamp(msg_body_pose, rel_T);
  msg_body_pose.header.stamp = stamp;
  msg_body_pose.header.frame_id = "camera_init";
  pub_pose.publish(msg_body_pose);

  return msg_body_pose;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;

  time_t rawtime = time(NULL);
  struct tm *timeinfo = localtime(&rawtime);
  char str_time[100];
  sprintf(str_time, "%04d%02d%02d%02d%02d", timeinfo->tm_year + 1900,
          timeinfo->tm_mon + 1, timeinfo->tm_mday, timeinfo->tm_hour,
          timeinfo->tm_min);
  std::string temp_str(str_time);
  time_str = temp_str;
  eskf_proc.eskf_params.time_str = time_str;

  nh.param<bool>("publish/path_en", path_en, true);
  nh.param<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
  nh.param<std::string>("common/imu_topic", imu_topic, "/livox/imu");
  nh.param<std::string>("common/gps_topic", gps_topic, "/gps");
  nh.param<std::string>("common/uwb_topic", uwb_topic,
                        "/nlink_linktrack_nodeframe2");
  nh.param<std::string>("common/vicon_topic", vicon_topic,
                        "/vrpn_client_node/uwb_ngx_tag/pose");
  nh.param<std::string>("common/mag_topic", mag_topic, "/imu/mag");
  nh.param<std::string>("common/wmm_cof_path", wmm_cof_path,
                        "src/igo_eskf/config/WMM.COF");
  nh.param<std::string>("common/file_save_path", file_save_path,
                        "/home/xng/ws_fusion_uwb/src/igo_eskf/data/res/");

  nh.param<bool>("common/en_vicon", en_vicon, false);
  nh.param<bool>("common/en_debug", en_debug, false);
  nh.param<bool>("common/en_time_sync", en_time_sync, true);
  eskf_proc.eskf_params.save_path = file_save_path;
  eskf_proc.eskf_params.en_debug = en_debug;

  nh.param<vector<double>>("params/init_bias/gyro",
                           eskf_proc.eskf_params.init_gyro_bias,
                           vector<double>());
  nh.param<vector<double>>("params/init_bias/accel",
                           eskf_proc.eskf_params.init_accel_bias,
                           vector<double>());
  nh.param<vector<double>>("params/init_bias/mag",
                           eskf_proc.eskf_params.init_mag_bias,
                           vector<double>());

  nh.param<vector<double>>(
      "params/cov/posi", eskf_proc.eskf_params.posi_cov_init, vector<double>());
  nh.param<vector<double>>(
      "params/cov/vel", eskf_proc.eskf_params.velo_cov_init, vector<double>());
  nh.param<vector<double>>("params/cov/ori", eskf_proc.eskf_params.ori_cov_init,
                           vector<double>());
  nh.param<vector<double>>("params/cov/gyro_bias",
                           eskf_proc.eskf_params.gyro_cov_init,
                           vector<double>());
  nh.param<vector<double>>("params/cov/acc_bias",
                           eskf_proc.eskf_params.accel_cov_init,
                           vector<double>());
  nh.param<vector<double>>("params/cov/mag_bias",
                           eskf_proc.eskf_params.mag_cov_init,
                           vector<double>());
  nh.param<vector<double>>("params/cov/g_bias",
                           eskf_proc.eskf_params.g_cov_init, vector<double>());
  nh.param<vector<double>>("params/process/vel",
                           eskf_proc.eskf_params.vel_mot_noise,
                           vector<double>());
  nh.param<vector<double>>("params/process/rot",
                           eskf_proc.eskf_params.rot_mot_noise,
                           vector<double>());
  nh.param<vector<double>>("params/process/gyro",
                           eskf_proc.eskf_params.gyro_mot_noise,
                           vector<double>());
  nh.param<vector<double>>("params/process/accel",
                           eskf_proc.eskf_params.accel_mot_noise,
                           vector<double>());
  nh.param<vector<double>>("params/process/mag",
                           eskf_proc.eskf_params.mag_mot_noise,
                           vector<double>());
  nh.param<vector<double>>("params/measurement/posi",
                           eskf_proc.eskf_params.posi_meas_noise,
                           vector<double>());
  nh.param<vector<double>>("params/measurement/vel",
                           eskf_proc.eskf_params.vel_meas_noise,
                           vector<double>());
  nh.param<vector<double>>("params/measurement/mag",
                           eskf_proc.eskf_params.ori_meas_noise,
                           vector<double>());

  nh.param<double>("options/local_gravity", gravity, 9.79484197226504);
  nh.param<bool>("options/en_sensor_init", en_sensor_init, true);
  nh.param<double>("options/init_duration", init_duration, 10);
  nh.param<double>("options/imu_filter_n_sigma", imu_filter_n_sigma, 3);
  nh.param<int>("options/imu_slide_window_size", imu_slide_window_size, 10);
  nh.param<bool>("options/en_rtk_vel", en_rtk_vel, false);
  nh.param<double>("options/zero_gyro_threshold", zero_gyro_threshold, 0.5);

  nh.param<std::string>("publish/pose_topic", pose_topic, "/fusion_pose");
  nh.param<std::string>("publish/path_topic", path_topic, "/fusion_path");
  nh.param<std::string>("publish/odom_topic", odom_topic, "/fusion_odom");

  ROS_INFO("eskf_proc.eskf_params.init_accel_bias: %f %f %f",
           eskf_proc.eskf_params.init_accel_bias[0],
           eskf_proc.eskf_params.init_accel_bias[1],
           eskf_proc.eskf_params.init_accel_bias[2]);
  ROS_INFO("eskf_proc.eskf_params.init_gyro_bias: %f %f %f",
           eskf_proc.eskf_params.init_gyro_bias[0],
           eskf_proc.eskf_params.init_gyro_bias[1],
           eskf_proc.eskf_params.init_gyro_bias[2]);
  ROS_INFO("INPUT: imu_topic: %s, gps_topic: %s", imu_topic.c_str(),
           gps_topic.c_str());
  ROS_INFO("OUTPUT: pose topic: %s, odom topic: %s, path topic: %s",
           pose_topic.c_str(), odom_topic.c_str(), path_topic.c_str());
  ROS_INFO("OUTPUT: zero_gyro_threshold: %f", zero_gyro_threshold);
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "camera_init";

  ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
  ros::Subscriber sub_gps = nh.subscribe(gps_topic, 200000, rtk_cbk);
  ros::Subscriber sub_mag = nh.subscribe(mag_topic, 200000, mavros_mag_cbk);
  ros::Subscriber sub_vicon = nh.subscribe(vicon_topic, 200000, vicon_cbk);

  // ros::Subscriber sub_uwb = nh.subscribe(uwb_topic, 200000, uwb_cbk);

  ros::Publisher pubOdomAftMapped =
      nh.advertise<nav_msgs::Odometry>(odom_topic, 100000);
  ros::Publisher pubPath = nh.advertise<nav_msgs::Path>(path_topic, 100000);
  ros::Publisher pubpose =
      nh.advertise<geometry_msgs::PoseStamped>(pose_topic, 100000);

  ros::Rate rate(5000);
  bool status = ros::ok();
  size_t point_num = 0;
  int count_ = 0;
  int pred_num = 0;
  int correct_num = 0;
  int debug_num = 0;

  if (en_sensor_init) {
    ROS_WARN("========== Sensor Initializing ...... ==========");
  }
  // std::shared_ptr<void> eskf_proc;
  // if (en_rtk_vel) {
  //   eskf_proc = std::make_shared<ESKF_VEL>();
  // } else {
  //   eskf_proc = std::make_shared<ESKF>();
  // }

  while (status) {
    auto clock1 = std::chrono::steady_clock::now();
    ros::spinOnce();

    double curr_init_stamp = get_stamp();
    if (en_sensor_init) {
      if (!is_sensor_init) {
        if (curr_init_stamp - init_init_stamp > init_duration &&
            imu_buffer.size() > 100) {
          if (sensor_init(imu_buffer, mavros_mag_buffer, acc_offset, gyr_offset,
                          std_mag, init_mag_heading)) {
            eskf_proc.eskf_params.init_heading = init_mag_heading;
            ROS_INFO("========== Sensor Initialize DONE ==========");
            imu_buffer.clear();
            gps_buffer.clear();
            is_sensor_init = true;
            is_mag_heading_init = true;
          }
        }
      } else {
        while (!imu_buffer.empty() && !gps_buffer.empty()) {
          // trim imu_buffer and gps_buffer

          sensor_msgs::Imu::Ptr curr_imu_data(
              new sensor_msgs::Imu(*imu_buffer.front()));
          // GPSGroup curr_uwb_data = uwb_buffer.front();
          GPSGroup curr_gps_data = gps_buffer.front();
          if (!eskf_proc.flg_eskf_init && !gps_buffer.empty()) {
            eskf_proc.Init(curr_imu_data, gps_buffer.front());
            if (!gps_buffer.empty()) {
              gps_buffer.pop_front();
            }
          }
          if (curr_imu_data->header.stamp.toSec() < curr_gps_data.timestamp) {
            eskf_proc.predict(curr_imu_data);
            imu_buffer.pop_front();
            pred_num++;
            double curr_stamp = 0.0;
            eskf_proc.get_pose(res_pos, res_quat, curr_stamp);
            // eskf_proc.get_vel(res_vel);
            geometry_msgs::PoseStamped pred_pose =
                publish_pose(pubpose, ros::Time().fromSec(curr_stamp));
            // if (path_en) publish_path(pubPath,
            // ros::Time().fromSec(curr_stamp));
            if (en_debug) {
              std::string write_path =
                  file_save_path + "predict_pose_" + time_str + ".txt";
              std::ofstream outfile;
              outfile.open(write_path, std::ofstream::app);
              outfile << setprecision(19) << curr_stamp << " "
                      << pred_pose.pose.position.x << " "
                      << pred_pose.pose.position.y << " "
                      << pred_pose.pose.position.z << " "
                      << pred_pose.pose.orientation.x << " "
                      << pred_pose.pose.orientation.y << " "
                      << pred_pose.pose.orientation.z << " "
                      << pred_pose.pose.orientation.w << std::endl;
              // std::string write_path2 =
              //     file_save_path + "predict_vel_" + time_str + ".txt";
              // std::ofstream outfile2;
              // outfile2.open(write_path2, std::ofstream::app);
              // outfile2 << setprecision(19) << curr_stamp << " " << res_vel[0]
              //          << " " << res_vel[1] << " " << res_vel[2] << " " << 0
              //          << " " << 0 << " " << 0 << " " << 1 << std::endl;
              // ROS_INFO("pose: %f %f %f %f %f %f %f %f", curr_stamp,
              // res_pos[0],
              //          res_pos[1], res_pos[2], res_quat.x(),
              // res_quat.y(),
              //          res_quat.z(), res_quat.w());
            }

          } else {
            correct_num++;
            eskf_proc.predict(curr_imu_data);
            imu_buffer.pop_front();
            eskf_proc.correct(curr_gps_data);
            // uwb_buffer.pop_front();
            if (!gps_buffer.empty()) {
              gps_buffer.pop_front();
            }

            double curr_stamp = 0.0;
            eskf_proc.get_pose(res_pos, res_quat, curr_stamp);
            eskf_proc.get_vel(res_vel);
            if (is_1st_pose) {
              init_pose = res2isometry();
              is_1st_pose = false;
            }
            geometry_msgs::PoseStamped correct_pose =
                publish_pose(pubpose, ros::Time().fromSec(curr_stamp));
            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped);
            /******* Publish path *******/
            if (path_en) publish_path(pubPath, ros::Time().fromSec(curr_stamp));

            std::string write_path =
                file_save_path + "fusion_pose_" + time_str + ".txt";
            std::ofstream outfile;
            outfile.open(write_path, std::ofstream::app);
            outfile << setprecision(19) << curr_stamp << " "
                    << correct_pose.pose.position.x << " "
                    << correct_pose.pose.position.y << " "
                    << correct_pose.pose.position.z << " "
                    << correct_pose.pose.orientation.x << " "
                    << correct_pose.pose.orientation.y << " "
                    << correct_pose.pose.orientation.z << " "
                    << correct_pose.pose.orientation.w << std::endl;
            outfile.close();
            std::string write_path2 =
                file_save_path + "predict_vel_" + time_str + ".txt";
            std::ofstream outfile2;
            outfile2.open(write_path2, std::ofstream::app);
            outfile2 << setprecision(19) << curr_stamp << " " << res_vel[0]
                     << " " << res_vel[1] << " " << res_vel[2] << " " << 0
                     << " " << 0 << " " << 0 << " " << 1 << std::endl;
            outfile2.close();
            count_++;

            std::cout << " pred_num: " << pred_num
                      << "  correct_num: " << correct_num << std::endl;
          }
        }
      }
    } else {
      while (!imu_buffer.empty() && !gps_buffer.empty()) {
        sensor_msgs::Imu::Ptr curr_imu_data(
            new sensor_msgs::Imu(*imu_buffer.front()));
        // GPSGroup curr_uwb_data = uwb_buffer.front();
        GPSGroup curr_gps_data = gps_buffer.front();

        // ===============DEBUG===============
        // std::string write_path_2 =
        //     "/home/xng/catkin_ws/src/inno_ligo/data/res/gps_in.txt";
        // std::ofstream outfile_2;
        // outfile_2.open(write_path_2, std::ofstream::app);
        // outfile_2 << debug_num << " " << curr_gps_data[1] << " "
        //           << curr_gps_data[2] << " " << curr_gps_data[3] << " " <<
        //           0
        //           << " " << 0 << " " << 0 << " " << 1 << std::endl;
        // outfile_2.close();

        // debug_num++;
        // ====================================

        if (!eskf_proc.flg_eskf_init && !gps_buffer.empty() &&
            is_mag_heading_init) {
          eskf_proc.Init(curr_imu_data, gps_buffer.front());
          if (!gps_buffer.empty()) {
            gps_buffer.pop_front();
          }
        }
        if (is_mag_heading_init) {
          if (curr_imu_data->header.stamp.toSec() < curr_gps_data.timestamp) {
            eskf_proc.predict(curr_imu_data);
            imu_buffer.pop_front();
            pred_num++;

            // =========== SAVE =========== //
            double curr_stamp = 0.0;
            eskf_proc.get_pose(res_pos, res_quat, curr_stamp);
            eskf_proc.get_vel(res_vel);
            publish_pose(pubpose, ros::Time().fromSec(curr_stamp));
            if (path_en) publish_path(pubPath, ros::Time().fromSec(curr_stamp));

            std::string write_path =
                file_save_path + "predict_pose_" + time_str + ".txt";
            std::ofstream outfile;
            outfile.open(write_path, std::ofstream::app);
            outfile << setprecision(19) << curr_stamp << " " << res_pos[0]
                    << " " << res_pos[1] << " " << res_pos[2] << " "
                    << res_quat.x() << " " << res_quat.y() << " "
                    << res_quat.z() << " " << res_quat.w() << std::endl;
            outfile.close();
            std::string write_path2 =
                file_save_path + "predict_vel_" + time_str + ".txt";
            std::ofstream outfile2;
            outfile2.open(write_path2, std::ofstream::app);
            outfile2 << setprecision(19) << curr_stamp << " " << res_vel[0]
                     << " " << res_vel[1] << " " << res_vel[2] << " " << 0
                     << " " << 0 << " " << 0 << " " << 1 << std::endl;
            ROS_INFO("pose: %f %f %f %f %f %f %f %f", curr_stamp, res_pos[0],
                     res_pos[1], res_pos[2], res_quat.x(), res_quat.y(),
                     res_quat.z(), res_quat.w());
          } else {
            // ROS_INFO("=====imu time: %f, gps time: %f",
            // curr_imu_data->header.stamp.toSec(),
            //          curr_gps_data.timestamp);
            correct_num++;
            eskf_proc.predict(curr_imu_data);
            imu_buffer.pop_front();
            eskf_proc.correct(curr_gps_data);
            // uwb_buffer.pop_front();
            if (!gps_buffer.empty()) {
              gps_buffer.pop_front();
            }

            double curr_stamp = 0.0;
            eskf_proc.get_pose(res_pos, res_quat, curr_stamp);
            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped);
            /******* Publish path *******/
            if (path_en) publish_path(pubPath, ros::Time().fromSec(curr_stamp));

            std::string write_path =
                file_save_path + "fusion_pose_" + time_str + ".txt";
            std::ofstream outfile;
            outfile.open(write_path, std::ofstream::app);
            outfile << setprecision(19) << curr_stamp << " " << res_pos[0]
                    << " " << res_pos[1] << " " << res_pos[2] << " "
                    << res_quat.x() << " " << res_quat.y() << " "
                    << res_quat.z() << " " << res_quat.w() << std::endl;
            outfile.close();
            count_++;

            std::cout << " pred_num: " << pred_num
                      << "  correct_num: " << correct_num << std::endl;
          }
        }
      }
    }

    status = ros::ok();
    rate.sleep();
  }
  return 0;
}
