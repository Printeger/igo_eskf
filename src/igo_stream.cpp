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
// double deg2rad = M_PI / 180.0;

std::mutex mtx_buffer;
condition_variable sig_buffer;

ofstream fout_pre, fout_out, fout_dbg;
double solve_time = 0, solve_const_H_time = 0;
double res_mean_last = 0.05, total_residual = 0.0; // 设置残差平均值，残差总和
double gyr_cov = 1.0e-5, acc_cov = 1.0e-4, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
// double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;

// double prior_cov_pos = 1.0e-4, prior_cov_vel = 1.0e-4, prior_cov_ori
// = 1.0e-6,
//        prior_cov_epsilon = 1.0e-6, prior_cov_delta = 1.0e-6;
// double meas_cov_pos = 10;
// bool flg_eskf_init = false;

int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, count_ = 0;
double last_timestamp_imu = -1.0, last_timestamp_gps = 0, first_gps_time = 0.0, last_timestamp_uwb, gps_curr_time = 0.0;
deque<double> time_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
// deque<V4D> gps_buffer;
deque<GPSGroup> gps_buffer;
deque<GPSGroup> uwb_buffer;
deque<geometry_msgs::PoseStamped> vicon_buffer;

bool flg_first_gps = true, path_en = true, flg_EKF_inited, en_vicon = false, en_debug = false;
std::string imu_topic, gps_topic, uwb_topic, vicon_topic, file_save_path;
bool TRANSAXIS = true;

vector<double> extrinT(3, 0.0);
vector<double> extrinR(9, 0.0);

// GPS with respect to IMU
V3D GPS_T_wrt_IMU(Zero3d);
M3D GPS_R_wrt_IMU(Eye3d);
V3D euler_cur;
V3D res_pos;
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

uint64_t convertGpsToUnix(uint32_t gpsWeek, uint32_t gpsTow)
{
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

void NED2ENU(const sensor_msgs::Imu::ConstPtr &imu_in, sensor_msgs::Imu::Ptr &imu_out)
{
    imu_out->header.stamp = imu_in->header.stamp;
    imu_out->header.frame_id = imu_in->header.frame_id;
    imu_out->linear_acceleration.x = imu_in->linear_acceleration.x * gravity;
    imu_out->linear_acceleration.y = -imu_in->linear_acceleration.y * gravity;
    imu_out->linear_acceleration.z = -imu_in->linear_acceleration.z * gravity;
    // rad/s
    imu_out->angular_velocity.x = imu_in->angular_velocity.x;
    imu_out->angular_velocity.y = -imu_in->angular_velocity.y;
    imu_out->angular_velocity.z = -imu_in->angular_velocity.z;

    if (en_debug)
    {
        std::string write_path1 = file_save_path + "acc_in.txt";
        std::ofstream outfile1;
        outfile1.open(write_path1, std::ofstream::app);
        outfile1 << setprecision(19) << imu_out->header.stamp.toSec() << " " << imu_out->linear_acceleration.x << " "
                 << imu_out->linear_acceleration.y << " " << imu_out->linear_acceleration.z << " " << 0 << " " << 0
                 << " " << 0 << " " << 1 << std::endl;
        outfile1.close();
        std::string write_path2 = file_save_path + "vel_in.txt";
        std::ofstream outfile2;
        outfile2.open(write_path2, std::ofstream::app);
        outfile2 << setprecision(19) << imu_out->header.stamp.toSec() << " " << imu_out->angular_velocity.x << " "
                 << imu_out->angular_velocity.y << " " << imu_out->angular_velocity.z << " " << 0 << " " << 0 << " "
                 << 0 << " " << 1 << std::endl;
        outfile2.close();
    }
}

void vicon_cbk(const geometry_msgs::PoseStamped::ConstPtr &msg_in)
{
    mtx_buffer.lock();
    geometry_msgs::PoseStamped tmp_msg = *msg_in;
    vicon_buffer.emplace_back(tmp_msg);
    mtx_buffer.unlock();
}

void uwb_cbk(const common_msgs::LinktrackNodeframe2::ConstPtr &uwb_msg)
{
    common_msgs::LinktrackNodeframe2::Ptr msg(new common_msgs::LinktrackNodeframe2(*uwb_msg));
    double timestamp = uwb_msg->local_time * 1e-6;
    mtx_buffer.lock();

    if (timestamp < last_timestamp_uwb)
    {
        ROS_WARN("uwb loop back, clear buffer");
        uwb_buffer.clear();
    }
    if (en_vicon)
    {
        double min_duration = 1e19;
        double duration = 0.0;
        // Find the vicon pose with the closest timestamp
        geometry_msgs::PoseStamped closest_pose;
        auto iter = vicon_buffer.begin();
        if (!vicon_buffer.empty())
        {
            closest_pose = vicon_buffer.front();
            // min_duration = std::abs(input.header.stamp.toSec() - closest_pose->header.stamp.toSec());
            for (auto it = vicon_buffer.begin(); it != vicon_buffer.end(); ++it)
            {
                duration = std::abs(timestamp - it->header.stamp.toSec());
                if (duration < min_duration)
                {
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
    if (!uwb_proc->proj_init)
    {
        uwb_proc->Initialize(msg->pos_3d.at(0), msg->pos_3d.at(1), msg->pos_3d.at(2));
        ROS_INFO("===== UWB initialize DONE =====");
    }
    uwb_proc->uwb_process(msg, temp_uwb);
    last_timestamp_uwb = timestamp;
    time_buffer.push_back(timestamp);
    uwb_buffer.push_back(temp_uwb);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void gps_cbk(const sensor_msgs::NavSatFix::ConstPtr &gps_msg)
{
    ROS_WARN("gps_cbk");
    sensor_msgs::NavSatFix::Ptr msg(new sensor_msgs::NavSatFix(*gps_msg));
    double timestamp = gps_msg->header.stamp.toSec();
    mtx_buffer.lock();

    if (timestamp < last_timestamp_gps)
    {
        ROS_WARN("gps loop back, clear buffer");
        gps_buffer.clear();
    }
    GPSGroup temp_utm;
    // V4D temp_utm;
    // to utm
    if (!gps_proc->proj_init)
    {
        gps_proc->Initialize(gps_msg->longitude, gps_msg->latitude, gps_msg->altitude);
    }
    gps_proc->Process(gps_msg, temp_utm);

    if (en_debug)
    {
        std::string write_path = file_save_path + "utm_enu_" + time_str + ".txt";
        std::ofstream outfile;
        outfile.open(write_path, std::ofstream::app);
        outfile << setprecision(19) << temp_utm.timestamp << " " << temp_utm.UTM[0] << " " << temp_utm.UTM[1] << " "
                << temp_utm.UTM[2] << " " << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;
        outfile.close();

        std::string write_path2 = file_save_path + "lla_" + time_str + ".txt";
        std::ofstream outfile2;
        outfile2.open(write_path2, std::ofstream::app);
        outfile2 << setprecision(19) << temp_utm.timestamp << " " << temp_utm.LLA[0] << " " << temp_utm.LLA[1] << " "
                 << temp_utm.LLA[2] << " " << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;
        outfile2.close();
    }

    last_timestamp_gps = timestamp;
    time_buffer.push_back(timestamp);
    gps_buffer.push_back(temp_utm);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void gps_cbk_vel(const gnss_comm::GnssPVTSolnMsg::ConstPtr &gps_msg)
{
    ROS_WARN("gps_cbk");
    gnss_comm::GnssPVTSolnMsg::Ptr msg(new gnss_comm::GnssPVTSolnMsg(*gps_msg));
    uint64_t recv_stamp = convertGpsToUnix(msg->time.week, msg->time.tow);

    ROS_INFO("recv_stamp: %ld, msg->time.week: %ld,  msg->time.tow: %ld", recv_stamp, msg->time.week, msg->time.tow);
    double timestamp = static_cast<double>(recv_stamp) - 8.5; // diff between imu and gps
    msg->vel_acc = timestamp;                                 // using vel_acc to store timestamp

    mtx_buffer.lock();

    if (timestamp < last_timestamp_gps)
    {
        ROS_WARN("gps loop back, clear buffer");
        gps_buffer.clear();
    }
    GPSGroup temp_utm;
    // V4D temp_utm;
    // to utm
    if (!gps_proc->proj_init)
    {
        gps_proc->Initialize(gps_msg->longitude, gps_msg->latitude, gps_msg->altitude);
    }
    gps_proc->Process(msg, temp_utm);

    if (en_debug)
    {
        std::string write_path = file_save_path + "utm_enu_" + time_str + ".txt";
        std::ofstream outfile;
        outfile.open(write_path, std::ofstream::app);
        outfile << setprecision(19) << temp_utm.timestamp << " " << temp_utm.UTM[0] << " " << temp_utm.UTM[1] << " "
                << temp_utm.UTM[2] << " " << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;
        outfile.close();

        std::string write_path2 = file_save_path + "lla_" + time_str + ".txt";
        std::ofstream outfile2;
        outfile2.open(write_path2, std::ofstream::app);
        outfile2 << setprecision(19) << temp_utm.timestamp << " " << temp_utm.LLA[0] << " " << temp_utm.LLA[1] << " "
                 << temp_utm.LLA[2] << " " << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;
        outfile2.close();
    }

    last_timestamp_gps = timestamp;
    time_buffer.push_back(timestamp);
    gps_buffer.push_back(temp_utm);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
    double timestamp = msg->header.stamp.toSec();
    mtx_buffer.lock();
    // ======= DEBUG ======= //
    // std::string write_path_1 = "/home/mint/ws_fusion_uwb/src/inno_ligo/data/res/acc_in.txt";
    // std::ofstream outfile_1;
    // outfile_1.open(write_path_1, std::ofstream::app);
    // outfile_1 << setprecision(19) << msg->header.stamp.toSec() << " " << msg->linear_acceleration.x << " "
    //           << msg->linear_acceleration.y << " " << msg->linear_acceleration.z << " " << 0 << " " << 0 << " " << 0
    //           << " " << 1 << std::endl;
    // outfile_1.close();

    // double av_1 = msg->angular_velocity.x * rad2degree;
    // double av_2 = msg->angular_velocity.y * rad2degree;
    // double av_3 = msg->angular_velocity.z * rad2degree;
    // std::string write_path_2 = "/home/mint/ws_fusion_uwb/src/inno_ligo/data/res/gyro_in.txt";
    // std::ofstream outfile_2;
    // outfile_2.open(write_path_2, std::ofstream::app);
    // outfile_2 << setprecision(19) << msg->header.stamp.toSec() << " " << av_1 << " " << av_2 << " " << av_3 << " " <<
    // 0
    //           << " " << 0 << " " << 0 << " " << 1 << std::endl;
    // outfile_2.close();

    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }
    last_timestamp_imu = timestamp;
    sensor_msgs::Imu::Ptr temp_imu(new sensor_msgs::Imu(*msg_in));
    if (1)
    {
        NED2ENU(msg_in, temp_imu);
    }
    else
    {
        temp_imu = msg;
    }

    imu_buffer.push_back(temp_imu);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
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

// void h_shared_model_GIO(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
// {
//     ekfom_data.h_x = MatrixXd::Zero(3, 12);
//     ekfom_data.h.resize(3);
//     // ekfom_data.h_x.block<3, 3>(0, 9) = Eigen::Matrix3d::Identity();
//     ekfom_data.h_x.block<1, 12>(0, 0) << 1, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//     ekfom_data.h_x.block<1, 12>(1, 0) << 0, 1, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//     ekfom_data.h_x.block<1, 12>(2, 0) << 0, 0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
//     // TODO======CHECK======
//     for (int i = 0; i < 3; i++)
//     {
//         //* predict pos - utm pos
//         ekfom_data.h(i) = Measures.gps.front()[i + 1] - s.pos(i);
//     }
// }

// pub pose
template <typename T> void set_posestamp(T &out)
{
    out.pose.position.x = res_pos[0];
    out.pose.position.y = res_pos[1];
    out.pose.position.z = res_pos[2];
    out.pose.orientation.x = res_quat.x();
    out.pose.orientation.y = res_quat.y();
    out.pose.orientation.z = res_quat.z();
    out.pose.orientation.w = res_quat.w();
}

void publish_odometry(const ros::Publisher &pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(gps_curr_time); // ros::Time().fromSec(lidar_end_time);
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
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, odomAftMapped.pose.pose.position.y,
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "body"));
}

void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(gps_curr_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0)
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    time_t rawtime = time(NULL);
    struct tm *timeinfo = localtime(&rawtime);
    char str_time[100];
    sprintf(str_time, "%04d%02d%02d%02d%02d", timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
            timeinfo->tm_hour, timeinfo->tm_min);
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
    nh.param<std::string>("common/uwb_topic", uwb_topic, "/nlink_linktrack_nodeframe2");
    nh.param<std::string>("common/vicon_topic", vicon_topic, "/vrpn_client_node/uwb_ngx_tag/pose");
    nh.param<std::string>("common/file_save_path", file_save_path, "/home/xng/ws_fusion_uwb/src/igo_eskf/data/res/");

    nh.param<bool>("common/en_vicon", en_vicon, false);
    nh.param<bool>("common/en_debug", en_debug, false);

    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());
    nh.param<vector<double>>("covariance/noise/posi", cov_prior_pos, vector<double>());
    nh.param<vector<double>>("covariance/noise/vel", cov_prior_vel, vector<double>());
    nh.param<vector<double>>("covariance/noise/ori", cov_prior_ori, vector<double>());
    nh.param<vector<double>>("covariance/noise/gyro", cov_noise_gyro, vector<double>());
    nh.param<vector<double>>("covariance/noise/acc", cov_noise_acc, vector<double>());
    nh.param<vector<double>>("covariance/process/gyro", cov_proc_gyro, vector<double>());
    nh.param<vector<double>>("covariance/process/accel", cov_proc_acc, vector<double>());
    nh.param<vector<double>>("covariance/measurement/posi", cov_meas_pos, vector<double>());
    nh.param<vector<double>>("covariance/measurement/vel", cov_meas_vel, vector<double>());

    ROS_INFO("cov_prior_pos: %f %f %f", cov_prior_pos[0], cov_prior_pos[1], cov_prior_pos[2]);
    ROS_INFO("cov_prior_vel: %f %f %f", cov_prior_vel[0], cov_prior_vel[1], cov_prior_vel[2]);
    ROS_INFO("cov_prior_ori: %f %f %f", cov_prior_ori[0], cov_prior_ori[1], cov_prior_ori[2]);
    ROS_INFO("cov_noise_gyro: %f %f %f", cov_noise_gyro[0], cov_noise_gyro[1], cov_noise_gyro[2]);
    ROS_INFO("cov_noise_acc: %f %f %f", cov_noise_acc[0], cov_noise_acc[1], cov_noise_acc[2]);
    ROS_INFO("cov_proc_gyro: %f %f %f", cov_proc_gyro[0], cov_proc_gyro[1], cov_proc_gyro[2]);
    ROS_INFO("cov_proc_acc: %f %f %f", cov_proc_acc[0], cov_proc_acc[1], cov_proc_acc[2]);
    ROS_INFO("cov_meas_pos: %f %f %f", cov_meas_pos[0], cov_meas_pos[1], cov_meas_pos[2]);
    ROS_INFO("cov_meas_vel: %f %f %f", cov_meas_vel[0], cov_meas_vel[1], cov_meas_vel[2]);
    ROS_INFO("imu_topic: %s, gps_topic: %s", imu_topic.c_str(), gps_topic.c_str());

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "camera_init";

    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
    ros::Subscriber sub_gps = nh.subscribe(gps_topic, 200000, gps_cbk_vel);
    // ros::Subscriber sub_uwb = nh.subscribe(uwb_topic, 200000, uwb_cbk);
    // ros::Subscriber sub_vicon = nh.subscribe(vicon_topic, 200000, vicon_cbk);

    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
    ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 100000);

    ros::Rate rate(5000);
    bool status = ros::ok();
    size_t point_num = 0;
    int count_ = 0;
    int pred_num = 0;
    int correct_num = 0;
    int debug_num = 0;

    while (status)
    {
        auto clock1 = std::chrono::steady_clock::now();
        ros::spinOnce();
        //* 3. 对齐传感器输入，存入Measures，
        while (!imu_buffer.empty() && !gps_buffer.empty())
        {
            sensor_msgs::Imu::Ptr curr_imu_data(new sensor_msgs::Imu(*imu_buffer.front()));
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

            if (!eskf_proc.flg_eskf_init)
            {
                eskf_proc.Init(curr_imu_data, cov_prior_pos, cov_prior_vel, cov_prior_ori, cov_noise_gyro,
                               cov_noise_acc, cov_proc_gyro, cov_proc_acc, cov_meas_pos, cov_meas_vel);
                // eskf_proc.Init(curr_imu_data, cov_prior_pos, cov_prior_vel, cov_prior_ori, cov_noise_gyro,
                //                cov_noise_acc, cov_proc_gyro, cov_proc_acc, cov_meas_pos);
            }
            if (curr_imu_data->header.stamp.toSec() < curr_gps_data.timestamp)
            {
                // ROS_INFO("*****imu time: %f, gps time: %f", curr_imu_data->header.stamp.toSec(),
                //          curr_gps_data.timestamp);
                eskf_proc.Predict(curr_imu_data);
                imu_buffer.pop_front();
                pred_num++;
            }
            else
            {
                // ROS_INFO("=====imu time: %f, gps time: %f", curr_imu_data->header.stamp.toSec(),
                //          curr_gps_data.timestamp);
                correct_num++;
                eskf_proc.Predict(curr_imu_data);
                imu_buffer.pop_front();
                eskf_proc.Correct(curr_gps_data);
                // uwb_buffer.pop_front();
                gps_buffer.pop_front();

                double curr_stamp = 0.0;
                eskf_proc.GetPose(res_pos, res_quat, curr_stamp);
                /******* Publish odometry *******/
                publish_odometry(pubOdomAftMapped);
                /******* Publish path *******/
                if (path_en)
                    publish_path(pubPath);

                std::string write_path = file_save_path + "fusion_pose_" + time_str + ".txt";
                std::ofstream outfile;
                outfile.open(write_path, std::ofstream::app);
                outfile << setprecision(19) << curr_stamp << " " << res_pos[0] << " " << res_pos[1] << " " << res_pos[2]
                        << " " << res_quat.x() << " " << res_quat.y() << " " << res_quat.z() << " " << res_quat.w()
                        << std::endl;
                outfile.close();
                count_++;

                std::cout << " pred_num: " << pred_num << "  correct_num: " << correct_num << std::endl;
            }
        }
        status = ros::ok();
        rate.sleep();
    }
    return 0;
}
