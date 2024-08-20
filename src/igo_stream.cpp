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
#include <common_msgs/LinktrackNodeframe2.h>
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

#include "eskf.hpp"
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
double gyr_cov = 1.0e-5, acc_cov = 1.0e-4, b_gyr_cov = 0.0001,
       b_acc_cov = 0.0001;
double init_mag_heading = 0.0;
double curr_heading_vel = 0.0;
double curr_heading_angle = 0.0;
double last_mag_heading = 0.0;
double last_mag_stamp = 0.0;
double curr_mag_stamp = 0.0;
double curr_imu_stamp = 0.0;
double last_imu_stamp = 0.0;

// double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov =
// 0.0001;

// double prior_cov_pos = 1.0e-4, prior_cov_vel = 1.0e-4, prior_cov_ori
// = 1.0e-6,
//        prior_cov_epsilon = 1.0e-6, prior_cov_delta = 1.0e-6;
// double meas_cov_pos = 10;
// bool flg_eskf_init = false;

int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0,
    laserCloudValidNum = 0, count_ = 0;
double last_timestamp_imu = -1.0, last_timestamp_gps = 0, first_gps_time = 0.0,
       last_timestamp_uwb, gps_curr_time = 0.0;
deque<double> time_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
// deque<V4D> gps_buffer;
deque<GPSGroup> gps_buffer;
deque<GPSGroup> uwb_buffer;
deque<geometry_msgs::PoseStamped> vicon_buffer;
deque<sensor_msgs::MagneticField> mavros_mag_buffer;

bool flg_first_gps = true, path_en = true, flg_EKF_inited, en_vicon = false,
     en_debug = false, is_mag_heading_init = false;
std::string imu_topic, gps_topic, uwb_topic, vicon_topic, mag_topic,
    file_save_path;
bool TRANSAXIS = true;

vector<double> extrinT(3, 0.0);
vector<double> extrinR(9, 0.0);

// GPS with respect to IMU
V3D GPS_T_wrt_IMU(Zero3d);
M3D GPS_R_wrt_IMU(Eye3d);
V3D euler_cur;
V3D res_pos;
V3D res_vel;
V3D sum_acc(Zero3d), sum_gyr(Zero3d);
V3D mean_acc(Zero3d), mean_gyr(Zero3d);
int cnt_imu = 0;
Eigen::Quaterniond res_quat(0, 0, 0, 1);

// shared_ptr<ImuProcess> imu_proc(new ImuProcess());
shared_ptr<GPSProcess> gps_proc(new GPSProcess());
shared_ptr<GPSProcess> uwb_proc(new GPSProcess());

// shared_ptr<ESKF> eskf_proc(new ESKF());
ESKF eskf_proc;
// pointcloud & imu msg
MeasureGroup Measures;
// 状态，噪声维度，输入
// esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
// state_ikfom state_point;  // 状态
// world系下lidar坐标
vect3 pos_lid;

// OUTPUT
nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;

void format_imu(const sensor_msgs::Imu::ConstPtr &imu_in,
                sensor_msgs::Imu::Ptr &imu_out) {
  imu_out->header.stamp = imu_in->header.stamp;
  imu_out->header.frame_id = imu_in->header.frame_id;
  // acc_+: x: back, y: right, z: down
  // gyro_+:x: anti-clock, y:anti-clock z: anti_clock
  V3D acc_offset(0.0, -0.0, 0.0);
  V3D gyr_offset(-1.0 - 06, -1.446528460784526517e-06,
                 2.978023936997308257e-07);
  imu_out->linear_acceleration.x =
      (imu_in->linear_acceleration.x - acc_offset[0]);
  imu_out->linear_acceleration.y =
      (imu_in->linear_acceleration.y - acc_offset[1]);
  imu_out->linear_acceleration.z =
      imu_in->linear_acceleration.z - acc_offset[2];
  imu_out->angular_velocity.x = imu_in->angular_velocity.x;
  imu_out->angular_velocity.y = imu_in->angular_velocity.y;
  imu_out->angular_velocity.z = imu_in->angular_velocity.z;
  // T265
  // imu_out->linear_acceleration.x = imu_in->linear_acceleration.z;
  // imu_out->linear_acceleration.y = -imu_in->linear_acceleration.x;
  // imu_out->linear_acceleration.z = -imu_in->linear_acceleration.y;
  // imu_out->angular_velocity.x = imu_in->angular_velocity.z * degree2rad;
  // imu_out->angular_velocity.y = -imu_in->angular_velocity.x * degree2rad;
  // imu_out->angular_velocity.z = -imu_in->angular_velocity.y * degree2rad;
  if (en_debug) {
    cnt_imu++;
    sum_acc +=
        V3D(imu_out->linear_acceleration.x, imu_out->linear_acceleration.y,
            imu_out->linear_acceleration.z);
    sum_gyr += V3D(imu_out->angular_velocity.x, imu_out->angular_velocity.y,
                   imu_out->angular_velocity.z);
    mean_acc = sum_acc / cnt_imu;
    mean_gyr = sum_gyr / cnt_imu;
    std::string write_path1 = file_save_path + "acc_in.txt";
    std::ofstream outfile1;
    outfile1.open(write_path1, std::ofstream::app);
    outfile1 << setprecision(19) << imu_out->header.stamp.toSec() << " "
             << imu_out->linear_acceleration.x << " "
             << imu_out->linear_acceleration.y << " "
             << imu_out->linear_acceleration.z << " " << mean_acc[0] << " "
             << mean_acc[1] << " " << mean_acc[2] << " " << 1 << std::endl;
    outfile1.close();
    std::string write_path2 = file_save_path + "gyro_in.txt";
    std::ofstream outfile2;
    outfile2.open(write_path2, std::ofstream::app);
    outfile2 << setprecision(19) << imu_out->header.stamp.toSec() << " "
             << imu_out->angular_velocity.x << " "
             << imu_out->angular_velocity.y << " "
             << imu_out->angular_velocity.z << " " << mean_gyr[0] << " "
             << mean_gyr[1] << " " << mean_gyr[2] << " " << 1 << std::endl;
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

bool sync_mag_gps() {
  if (mavros_mag_buffer.empty() && !gps_buffer.empty()) {
    gps_buffer.pop_front();
    ROS_WARN("mag_buffer is empty");
    return false;
  }
  if (!mavros_mag_buffer.empty() && gps_buffer.empty()) {
    mavros_mag_buffer.pop_front();
    ROS_WARN("gps_buffer is empty");
    return false;
  }
  if (mavros_mag_buffer.empty() && gps_buffer.empty()) {
    ROS_WARN("mag_buffer and gps_buffer are empty");
    return false;
  }
  double gps_time = 0.0;
  if (!gps_buffer.empty() && !mavros_mag_buffer.empty()) {
    gps_time = gps_buffer.front().timestamp;
  }
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
    mavros_mag_buffer.erase(mavros_mag_buffer.begin(), iter);
  }

  if (!is_mag_heading_init) {
    init_mag_heading =
        atan2(closest_mag.magnetic_field.y, closest_mag.magnetic_field.x);
    curr_heading_angle = init_mag_heading;
    last_mag_heading = init_mag_heading;
    last_mag_stamp = closest_mag.header.stamp.toSec();
    is_mag_heading_init = true;
  } else {
    curr_heading_angle =
        atan2(closest_mag.magnetic_field.y, closest_mag.magnetic_field.x) -
        init_mag_heading;
    if (curr_heading_angle > M_PI) {
      curr_heading_angle -= 2 * M_PI;
    }
    if (curr_heading_angle < -M_PI) {
      curr_heading_angle += 2 * M_PI;
    }

    std::string write_path =
        file_save_path + "curr_heading" + time_str + ".txt";
    std::ofstream outfile;
    outfile.open(write_path, std::ofstream::app);
    outfile << setprecision(19) << closest_mag.header.stamp.toSec() << " "
            << curr_heading_angle << " " << curr_heading_angle << " "
            << curr_heading_angle << " " << 0 << " " << 0 << " " << 0 << " "
            << 1 << std::endl;
    outfile.close();

    curr_mag_stamp = closest_mag.header.stamp.toSec();
    curr_heading_vel = (curr_heading_angle - last_mag_heading) /
                       (curr_mag_stamp - last_mag_stamp);
  }
  gps_buffer.front().magnetic =
      V3D(init_mag_heading, 0.0, curr_heading_vel);  // TODO: ENU

  // Rotate GPS UTM by curr_heading_angle around the z-axis
  double cos_angle = cos(curr_heading_angle);
  double sin_angle = sin(curr_heading_angle);
  double x = gps_buffer.front().UTM[0];
  double y = gps_buffer.front().UTM[1];
  gps_buffer.front().UTM[0] = x * cos_angle - y * sin_angle;
  gps_buffer.front().UTM[1] = x * sin_angle + y * cos_angle;

  return true;
}

// void smoothImuData() {
//   // Check if there are enough IMU measurements in the buffer
//   if (imu_buffer.size() < WINDOW_SIZE) {
//     return;
//   }

//   // Initialize variables for smoothed acceleration and angular velocity
//   V3D smoothed_acc(Zero3d);
//   V3D smoothed_gyr(Zero3d);

//   // Calculate the average acceleration and angular velocity over the window
//   for (const auto &imu_msg : imu_buffer) {
//     smoothed_acc +=
//         V3D(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y,
//             imu_msg->linear_acceleration.z);
//     smoothed_gyr +=
//         V3D(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y,
//             imu_msg->angular_velocity.z);
//   }
//   smoothed_acc /= WINDOW_SIZE;
//   smoothed_gyr /= WINDOW_SIZE;

//   // Update the IMU measurements in the buffer with the smoothed values
//   for (auto &imu_msg : imu_buffer) {
//     imu_msg->linear_acceleration.x = smoothed_acc[0];
//     imu_msg->linear_acceleration.y = smoothed_acc[1];
//     imu_msg->linear_acceleration.z = smoothed_acc[2];
//     imu_msg->angular_velocity.x = smoothed_gyr[0];
//     imu_msg->angular_velocity.y = smoothed_gyr[1];
//     imu_msg->angular_velocity.z = smoothed_gyr[2];
//   }
// }

void NED2ENU(const sensor_msgs::Imu::ConstPtr &imu_in,
             sensor_msgs::Imu::Ptr &imu_out) {
  double time_diff_gps_imu = 10.0;
  imu_out->header.stamp =
      imu_in->header.stamp + ros::Duration(time_diff_gps_imu);
  // static stable 0808
  imu_out->header.frame_id = imu_in->header.frame_id;
  imu_out->linear_acceleration.x = imu_in->linear_acceleration.x;
  imu_out->linear_acceleration.y = -imu_in->linear_acceleration.y;
  imu_out->linear_acceleration.z = -imu_in->linear_acceleration.z;
  // rad/s
  imu_out->angular_velocity.x = imu_in->angular_velocity.x * degree2rad;
  imu_out->angular_velocity.y = imu_in->angular_velocity.y * degree2rad;
  imu_out->angular_velocity.z = imu_in->angular_velocity.z * degree2rad;
  // TODO: TEST
  // V3D acc_offset(-0.1069723210427511173, 0.1317078687453833996,
  // -0.0326215262); V3D
  // gyr_offset(0.0002170503748205379331, 2.706704711055054452e-05,
  //                -0.002261991278184427839);
  // imu_out->header.frame_id = imu_in->header.frame_id;
  // imu_out->linear_acceleration.x =
  //     -(imu_in->linear_acceleration.x - acc_offset[0]);
  // imu_out->linear_acceleration.y =
  //     (imu_in->linear_acceleration.y - acc_offset[1]);
  // imu_out->linear_acceleration.z =
  //     -(imu_in->linear_acceleration.z - acc_offset[2]);
  // // rad/s
  // imu_out->angular_velocity.x =
  //     (imu_in->angular_velocity.x - gyr_offset[0]) * degree2rad;
  // imu_out->angular_velocity.y =
  //     -(imu_in->angular_velocity.y - gyr_offset[1]) * degree2rad;
  // imu_out->angular_velocity.z =
  //     -(imu_in->angular_velocity.z - gyr_offset[2]) * degree2rad;

  if (en_debug) {
    cnt_imu++;
    sum_acc += V3D(imu_in->linear_acceleration.x, imu_in->linear_acceleration.y,
                   imu_in->linear_acceleration.z);
    sum_gyr += V3D(imu_in->angular_velocity.x, imu_in->angular_velocity.y,
                   imu_in->angular_velocity.z);
    mean_acc = sum_acc / cnt_imu;
    mean_gyr = sum_gyr / cnt_imu;
    std::string write_path1 = file_save_path + "acc_in_" + time_str + ".txt";
    std::ofstream outfile1;
    outfile1.open(write_path1, std::ofstream::app);
    outfile1 << setprecision(19) << imu_in->header.stamp.toSec() << " "
             << imu_in->linear_acceleration.x << " "
             << imu_in->linear_acceleration.y << " "
             << imu_in->linear_acceleration.z << " " << mean_acc[0] << " "
             << mean_acc[1] << " " << mean_acc[2] << " " << 1 << std::endl;
    outfile1.close();
    std::string write_path2 = file_save_path + "gyro_in" + time_str + ".txt";
    std::ofstream outfile2;
    outfile2.open(write_path2, std::ofstream::app);
    outfile2 << setprecision(19) << imu_in->header.stamp.toSec() << " "
             << imu_in->angular_velocity.x << " " << imu_in->angular_velocity.y
             << " " << imu_in->angular_velocity.z << " " << mean_gyr[0] << " "
             << mean_gyr[1] << " " << mean_gyr[2] << " " << 1 << std::endl;
    outfile2.close();
  }
}

void vicon_cbk(const geometry_msgs::PoseStamped::ConstPtr &msg_in) {
  mtx_buffer.lock();
  geometry_msgs::PoseStamped tmp_msg = *msg_in;
  vicon_buffer.emplace_back(tmp_msg);
  mtx_buffer.unlock();
}

void uwb_cbk(const common_msgs::LinktrackNodeframe2::ConstPtr &uwb_msg) {
  common_msgs::LinktrackNodeframe2::Ptr msg(
      new common_msgs::LinktrackNodeframe2(*uwb_msg));
  double timestamp = uwb_msg->local_time * 1e-6;
  mtx_buffer.lock();

  if (timestamp < last_timestamp_uwb) {
    ROS_WARN("uwb loop back, clear buffer");
    uwb_buffer.clear();
  }
  if (en_vicon) {
    double min_duration = 1e19;
    double duration = 0.0;
    // Find the vicon pose with the closest timestamp
    geometry_msgs::PoseStamped closest_pose;
    auto iter = vicon_buffer.begin();
    if (!vicon_buffer.empty()) {
      closest_pose = vicon_buffer.front();
      // min_duration = std::abs(input.header.stamp.toSec() -
      // closest_pose->header.stamp.toSec());
      for (auto it = vicon_buffer.begin(); it != vicon_buffer.end(); ++it) {
        duration = std::abs(timestamp - it->header.stamp.toSec());
        if (duration < min_duration) {
          min_duration = duration;
          closest_pose = *it;
          iter = it;
        }
      }
      vicon_buffer.erase(vicon_buffer.begin(), iter);
    }
    msg->pos_3d.at(2) = closest_pose.pose.position.z;
  }

  GPSGroup temp_uwb;
  if (!uwb_proc->proj_init) {
    uwb_proc->Initialize(msg->pos_3d.at(0), msg->pos_3d.at(1),
                         msg->pos_3d.at(2));
    ROS_INFO("===== UWB initialize DONE =====");
  }
  uwb_proc->uwb_process(msg, temp_uwb);
  last_timestamp_uwb = timestamp;
  time_buffer.push_back(timestamp);
  uwb_buffer.push_back(temp_uwb);
  mtx_buffer.unlock();
  sig_buffer.notify_all();
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

  if (en_debug) {
    std::string write_path = file_save_path + "utm_enu_" + time_str + ".txt";
    std::ofstream outfile;
    outfile.open(write_path, std::ofstream::app);
    outfile << setprecision(19) << temp_utm.timestamp << " " << temp_utm.UTM[0]
            << " " << temp_utm.UTM[1] << " " << temp_utm.UTM[2] << " " << 0
            << " " << 0 << " " << 0 << " " << 1 << std::endl;
    outfile.close();

    std::string write_path2 = file_save_path + "lla_" + time_str + ".txt";
    std::ofstream outfile2;
    outfile2.open(write_path2, std::ofstream::app);
    outfile2 << setprecision(19) << temp_utm.timestamp << " " << temp_utm.LLA[0]
             << " " << temp_utm.LLA[1] << " " << temp_utm.LLA[2] << " " << 0
             << " " << 0 << " " << 0 << " " << 1 << std::endl;
    outfile2.close();
  }

  last_timestamp_gps = timestamp;
  time_buffer.push_back(timestamp);
  gps_buffer.push_back(temp_utm);
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void gps_cbk_vel(const gnss_comm::GnssPVTSolnMsg::ConstPtr &gps_msg) {
  ROS_WARN("gps_cbk");
  gnss_comm::GnssPVTSolnMsg::Ptr msg(new gnss_comm::GnssPVTSolnMsg(*gps_msg));
  uint64_t recv_stamp = convertGpsToUnix(msg->time.week, msg->time.tow);

  ROS_INFO("recv_stamp: %ld, msg->time.week: %ld,  msg->time.tow: %ld",
           recv_stamp, msg->time.week, msg->time.tow);
  double timestamp =
      static_cast<double>(recv_stamp);  // diff between imu and gps
  msg->vel_acc = timestamp;             // using vel_acc to store timestamp

  mtx_buffer.lock();

  if (timestamp < last_timestamp_gps) {
    ROS_WARN("gps loop back, clear buffer");
    gps_buffer.clear();
  }
  GPSGroup temp_utm;
  if (!gps_proc->proj_init) {
    gps_proc->Initialize(gps_msg->longitude, gps_msg->latitude,
                         gps_msg->altitude);
  }
  gps_proc->Process(msg, temp_utm);

  if (en_debug) {
    std::string write_path = file_save_path + "utm_enu_" + time_str + ".txt";
    std::ofstream outfile;
    outfile.open(write_path, std::ofstream::app);
    outfile << setprecision(19) << temp_utm.timestamp << " " << temp_utm.UTM[0]
            << " " << temp_utm.UTM[1] << " " << temp_utm.UTM[2] << " " << 0
            << " " << 0 << " " << 0 << " " << 1 << std::endl;
    outfile.close();

    std::string write_path2 = file_save_path + "lla_" + time_str + ".txt";
    std::ofstream outfile2;
    outfile2.open(write_path2, std::ofstream::app);
    outfile2 << setprecision(19) << temp_utm.timestamp << " " << temp_utm.LLA[0]
             << " " << temp_utm.LLA[1] << " " << temp_utm.LLA[2] << " " << 0
             << " " << 0 << " " << 0 << " " << 1 << std::endl;
    outfile2.close();
  }

  last_timestamp_gps = timestamp;
  time_buffer.push_back(timestamp);
  gps_buffer.push_back(temp_utm);
  if (!sync_mag_gps()) {
    ROS_WARN("sync_mag_gps failed");
  }
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) {
  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
  double timestamp = msg->header.stamp.toSec();
  mtx_buffer.lock();
  // ======= DEBUG ======= //
  // std::string write_path_1 =
  // "/home/mint/ws_fusion_uwb/src/inno_ligo/data/res/acc_in.txt"; std::ofstream
  // outfile_1; outfile_1.open(write_path_1, std::ofstream::app); outfile_1 <<
  // setprecision(19) << msg->header.stamp.toSec() << " " <<
  // msg->linear_acceleration.x << " "
  //           << msg->linear_acceleration.y << " " <<
  //           msg->linear_acceleration.z << " " << 0 << " " << 0 << " " << 0
  //           << " " << 1 << std::endl;
  // outfile_1.close();

  // double av_1 = msg->angular_velocity.x * rad2degree;
  // double av_2 = msg->angular_velocity.y * rad2degree;
  // double av_3 = msg->angular_velocity.z * rad2degree;
  // std::string write_path_2 =
  // "/home/mint/ws_fusion_uwb/src/inno_ligo/data/res/gyro_in.txt";
  // std::ofstream outfile_2;
  // outfile_2.open(write_path_2, std::ofstream::app);
  // outfile_2 << setprecision(19) << msg->header.stamp.toSec() << " " << av_1
  // << " " << av_2 << " " << av_3 << " " <<
  // 0
  //           << " " << 0 << " " << 0 << " " << 1 << std::endl;
  // outfile_2.close();

  if (timestamp < last_timestamp_imu) {
    ROS_WARN("imu loop back, clear buffer");
    imu_buffer.clear();
  }
  last_timestamp_imu = timestamp;
  sensor_msgs::Imu::Ptr temp_imu(new sensor_msgs::Imu(*msg_in));
  if (0) {
    NED2ENU(msg_in, temp_imu);
  } else {
    format_imu(msg, temp_imu);
  }
  // curr_imu_stamp = temp_imu.header.stamp.toSec();// TODO: 0808 slide window
  // last_imu_stamp = temp_imu.header.stamp.toSec();
  // // Process IMUs in 50ms duration
  // if (time_buffer.size() >= 2) {
  //   double start_time = time_buffer.front();
  //   double end_time = time_buffer.back();
  //   if (end_time - start_time >= 0.05) {
  //     // Process IMUs in the 50ms duration
  //     std::vector<sensor_msgs::Imu::Ptr> imu_measurements;
  //     while (!imu_buffer.empty()) {
  //       double imu_timestamp = imu_buffer.front()->header.stamp.toSec();
  //       if (imu_timestamp >= start_time && imu_timestamp <= end_time) {
  //         imu_measurements.push_back(imu_buffer.front());
  //         imu_buffer.pop_front();
  //       } else {
  //         break;
  //       }
  //     }

  //     // Process the IMU measurements
  //     process_imu_measurements(imu_measurements);
  //   }
  // }

  imu_buffer.push_back(temp_imu);
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

// pixhawk mag: NED, Xsens: SWD
void mavros_mag_cbk(const sensor_msgs::MagneticField::ConstPtr &msg) {
  mtx_buffer.lock();
  sensor_msgs::MagneticField tmp_msg = *msg;
  double time_diff_gps_mag = 8.5;
  ros::Time new_stamp = tmp_msg.header.stamp + ros::Duration(time_diff_gps_mag);
  tmp_msg.header.stamp = new_stamp;

  mavros_mag_buffer.push_back(tmp_msg);
  mtx_buffer.unlock();

  // double curr_heading_angle = atan2(tmp_msg.vector.y,
  // tmp_msg.vector.x) - init_mag_heading; if (curr_heading_angle >
  // M_PI) {
  //   curr_heading_angle -= 2 * M_PI;
  // }
  // if (curr_heading_angle < -M_PI) {
  //   curr_heading_angle += 2 * M_PI;
  // }
  // sum_heading += curr_heading_angle;
  // cnt_++;
  // double mean_heading = sum_heading / cnt_;
  // std::string write_path = file_save_path + "mavros_" + time_str + ".txt";
  // std::ofstream outfile;
  // outfile.open(write_path, std::ofstream::app);
  // outfile << setprecision(19) << tmp_msg.header.stamp.toSec() << " " <<
  // curr_heading_angle << " " << mean_heading
  //         << " "
  //         << mean_heading << " " << 0 << " " << 0 << " " << 0 << " " << 1 <<
  //         std::endl;
  // // outfile.close();
}

// 离GPS时间点最近的IMU数据从缓存队列中取出，进行时间对齐，并保存到meas中
// bool sync_packages(MeasureGroup &meas) {
//   if (imu_buffer.empty() || gps_buffer.empty()) {
//     return false;
//   }
//   double gps_time = 0.0;
//   /*** push gps data, and pop from imu buffer ***/
//   meas.gps.clear();
//   while ((!gps_buffer.empty())) {
//     gps_time = gps_buffer.front().timestamp;
//     meas.gps.push_back(gps_buffer.front());
//     gps_buffer.pop_front();
//   }
//   gps_curr_time = gps_time;
//   /*** push imu data, and pop from imu buffer ***/
//   double imu_time = imu_buffer.front()->header.stamp.toSec();
//   meas.imu.clear();
//   while ((!imu_buffer.empty()) && (imu_time < gps_time)) {
//     imu_time = imu_buffer.front()->header.stamp.toSec();
//     if (imu_time > gps_time) break;
//     sensor_msgs::Imu::Ptr temp_imu(new
//     sensor_msgs::Imu(*imu_buffer.front()));
//     // NED2ENU(imu_buffer.front(), temp_imu);
//     meas.imu.push_back(temp_imu);
//     imu_buffer.pop_front();
//     // }
//   }
//   // std::cout << "-----------" << std::endl;
//   std::cout << " meas.imu.size: " << meas.imu.size()
//             << "    meas.gps.size: " << meas.gps.size() << std::endl;
//   time_buffer.pop_front();
//   return true;
// }

// void h_shared_model_GIO(state_ikfom &s, esekfom::dyn_share_datastruct<double>
// &ekfom_data)
// {
//     ekfom_data.h_x = MatrixXd::Zero(3, 12);
//     ekfom_data.h.resize(3);
//     // ekfom_data.h_x.block<3, 3>(0, 9) = Eigen::Matrix3d::Identity();
//     ekfom_data.h_x.block<1, 12>(0, 0) << 1, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0,
//     0.0, 0.0, 0.0, 0.0; ekfom_data.h_x.block<1, 12>(1, 0) << 0, 1, 0, 0.0,
//     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; ekfom_data.h_x.block<1, 12>(2, 0)
//     << 0, 0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//     // TODO======CHECK======
//     for (int i = 0; i < 3; i++)
//     {
//         //* predict pos - utm pos
//         ekfom_data.h(i) = Measures.gps.front()[i + 1] - s.pos(i);
//     }
// }

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

void publish_pose(const ros::Publisher pub_pose, ros::Time stamp) {
  set_posestamp(msg_body_pose);
  msg_body_pose.header.stamp = stamp;
  msg_body_pose.header.frame_id = "camera_init";
  pub_pose.publish(msg_body_pose);
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

  vector<double> cov_prior_pos(3, 0.0);
  vector<double> cov_prior_vel(3, 0.0);
  vector<double> cov_prior_ori(3, 0.0);
  vector<double> cov_noise_gyro(3, 0.0);
  vector<double> cov_noise_acc(3, 0.0);
  vector<double> cov_proc_gyro(3, 0.0);
  vector<double> cov_proc_acc(3, 0.0);
  vector<double> cov_meas_pos(3, 0.0);
  vector<double> cov_meas_vel(3, 0.0);

  nh.param<bool>("publish/path_en", path_en, true);
  nh.param<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
  nh.param<std::string>("common/imu_topic", imu_topic, "/livox/imu");
  nh.param<std::string>("common/gps_topic", gps_topic, "/gps");
  nh.param<std::string>("common/uwb_topic", uwb_topic,
                        "/nlink_linktrack_nodeframe2");
  nh.param<std::string>("common/vicon_topic", vicon_topic,
                        "/vrpn_client_node/uwb_ngx_tag/pose");
  nh.param<std::string>("common/mag_topic", mag_topic, "/imu/mag");
  nh.param<std::string>("common/file_save_path", file_save_path,
                        "/home/xng/ws_fusion_uwb/src/igo_eskf/data/res/");

  nh.param<bool>("common/en_vicon", en_vicon, false);
  nh.param<bool>("common/en_debug", en_debug, false);

  nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
  nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());
  nh.param<vector<double>>("covariance/noise/posi", cov_prior_pos,
                           vector<double>());
  nh.param<vector<double>>("covariance/noise/vel", cov_prior_vel,
                           vector<double>());
  nh.param<vector<double>>("covariance/noise/ori", cov_prior_ori,
                           vector<double>());
  nh.param<vector<double>>("covariance/noise/gyro", cov_noise_gyro,
                           vector<double>());
  nh.param<vector<double>>("covariance/noise/acc", cov_noise_acc,
                           vector<double>());
  nh.param<vector<double>>("covariance/process/gyro", cov_proc_gyro,
                           vector<double>());
  nh.param<vector<double>>("covariance/process/accel", cov_proc_acc,
                           vector<double>());
  nh.param<vector<double>>("covariance/measurement/posi", cov_meas_pos,
                           vector<double>());
  nh.param<vector<double>>("covariance/measurement/vel", cov_meas_vel,
                           vector<double>());

  ROS_INFO("cov_prior_pos: %f %f %f", cov_prior_pos[0], cov_prior_pos[1],
           cov_prior_pos[2]);
  ROS_INFO("cov_prior_vel: %f %f %f", cov_prior_vel[0], cov_prior_vel[1],
           cov_prior_vel[2]);
  ROS_INFO("cov_prior_ori: %f %f %f", cov_prior_ori[0], cov_prior_ori[1],
           cov_prior_ori[2]);
  ROS_INFO("cov_noise_gyro: %f %f %f", cov_noise_gyro[0], cov_noise_gyro[1],
           cov_noise_gyro[2]);
  ROS_INFO("cov_noise_acc: %f %f %f", cov_noise_acc[0], cov_noise_acc[1],
           cov_noise_acc[2]);
  ROS_INFO("cov_proc_gyro: %f %f %f", cov_proc_gyro[0], cov_proc_gyro[1],
           cov_proc_gyro[2]);
  ROS_INFO("cov_proc_acc: %f %f %f", cov_proc_acc[0], cov_proc_acc[1],
           cov_proc_acc[2]);
  ROS_INFO("cov_meas_pos: %f %f %f", cov_meas_pos[0], cov_meas_pos[1],
           cov_meas_pos[2]);
  ROS_INFO("cov_meas_vel: %f %f %f", cov_meas_vel[0], cov_meas_vel[1],
           cov_meas_vel[2]);
  ROS_INFO("imu_topic: %s, gps_topic: %s", imu_topic.c_str(),
           gps_topic.c_str());

  path.header.stamp = ros::Time::now();
  path.header.frame_id = "camera_init";

  ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
  ros::Subscriber sub_gps = nh.subscribe(gps_topic, 200000, gps_cbk_vel);
  //   ros::Subscriber sub_mag = nh.subscribe(mag_topic, 200000, mag_cbk);
  ros::Subscriber sub_mag = nh.subscribe(mag_topic, 200000, mavros_mag_cbk);

  // ros::Subscriber sub_uwb = nh.subscribe(uwb_topic, 200000, uwb_cbk);
  // ros::Subscriber sub_vicon = nh.subscribe(vicon_topic, 200000, vicon_cbk);

  ros::Publisher pubOdomAftMapped =
      nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
  ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 100000);
  ros::Publisher pubpose =
      nh.advertise<geometry_msgs::PoseStamped>("/fusion_pose", 100000);

  ros::Rate rate(5000);
  bool status = ros::ok();
  size_t point_num = 0;
  int count_ = 0;
  int pred_num = 0;
  int correct_num = 0;
  int debug_num = 0;

  while (status) {
    auto clock1 = std::chrono::steady_clock::now();
    ros::spinOnce();
    //* 3. 对齐传感器输入，存入Measures，
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
      //           << curr_gps_data[2] << " " << curr_gps_data[3] << " " << 0
      //           << " " << 0 << " " << 0 << " " << 1 << std::endl;
      // outfile_2.close();

      // debug_num++;
      // ====================================

      if (!eskf_proc.flg_eskf_init && !gps_buffer.empty() &&
          is_mag_heading_init) {
        eskf_proc.Init(curr_imu_data, gps_buffer.front(), cov_prior_pos,
                       cov_prior_vel, cov_prior_ori, cov_noise_gyro,
                       cov_noise_acc, cov_meas_pos, cov_proc_gyro,
                       cov_proc_acc);
        if (!gps_buffer.empty()) {
          gps_buffer.pop_front();
        }
      }
      if (is_mag_heading_init) {
        if (curr_imu_data->header.stamp.toSec() < curr_gps_data.timestamp) {
          // ROS_INFO("*****imu time: %f, gps time: %f",
          // curr_imu_data->header.stamp.toSec(),
          //          curr_gps_data.timestamp);
          eskf_proc.Predict(curr_imu_data);
          imu_buffer.pop_front();
          pred_num++;

          // =========== SAVE =========== //
          double curr_stamp = 0.0;
          eskf_proc.GetPose(res_pos, res_quat, curr_stamp);
          eskf_proc.GetVelocity(res_vel);
          publish_pose(pubpose, ros::Time().fromSec(curr_stamp));
          if (path_en) publish_path(pubPath, ros::Time().fromSec(curr_stamp));

          std::string write_path =
              file_save_path + "predict_pose_" + time_str + ".txt";
          std::ofstream outfile;
          outfile.open(write_path, std::ofstream::app);
          outfile << setprecision(19) << curr_stamp << " " << res_pos[0] << " "
                  << res_pos[1] << " " << res_pos[2] << " " << res_quat.x()
                  << " " << res_quat.y() << " " << res_quat.z() << " "
                  << res_quat.w() << std::endl;
          outfile.close();
          std::string write_path2 =
              file_save_path + "predict_vel_" + time_str + ".txt";
          std::ofstream outfile2;
          outfile2.open(write_path2, std::ofstream::app);
          outfile2 << setprecision(19) << curr_stamp << " " << res_vel[0] << " "
                   << res_vel[1] << " " << res_vel[2] << " " << 0 << " " << 0
                   << " " << 0 << " " << 1 << std::endl;
          ROS_INFO("pose: %f %f %f %f %f %f %f %f", curr_stamp, res_pos[0],
                   res_pos[1], res_pos[2], res_quat.x(), res_quat.y(),
                   res_quat.z(), res_quat.w());
        } else {
          // ROS_INFO("=====imu time: %f, gps time: %f",
          // curr_imu_data->header.stamp.toSec(),
          //          curr_gps_data.timestamp);
          correct_num++;
          eskf_proc.Predict(curr_imu_data);
          imu_buffer.pop_front();
          eskf_proc.Correct(curr_gps_data);
          // uwb_buffer.pop_front();
          if (!gps_buffer.empty()) {
            gps_buffer.pop_front();
          }

          double curr_stamp = 0.0;
          eskf_proc.GetPose(res_pos, res_quat, curr_stamp);
          /******* Publish odometry *******/
          publish_odometry(pubOdomAftMapped);
          /******* Publish path *******/
          if (path_en) publish_path(pubPath, ros::Time().fromSec(curr_stamp));

          std::string write_path =
              file_save_path + "fusion_pose_" + time_str + ".txt";
          std::ofstream outfile;
          outfile.open(write_path, std::ofstream::app);
          outfile << setprecision(19) << curr_stamp << " " << res_pos[0] << " "
                  << res_pos[1] << " " << res_pos[2] << " " << res_quat.x()
                  << " " << res_quat.y() << " " << res_quat.z() << " "
                  << res_quat.w() << std::endl;
          outfile.close();
          count_++;

          std::cout << " pred_num: " << pred_num
                    << "  correct_num: " << correct_num << std::endl;
        }
      }
    }

    status = ros::ok();
    rate.sleep();
  }
  return 0;
}
