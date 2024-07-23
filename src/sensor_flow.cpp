// #include "sensor_flow.hpp"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

#include <chrono>
#include <condition_variable>
#include <csignal>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "../../devel/include/inno_ligo/sensor_measures.h"
#include "../../devel/include/inno_ligo/sensor_status.h"
#include "threads_control.hpp"

std::shared_ptr<StateMonitor> SBits;
std::string lidar_topic, imu_topic, gps_topic;
std::mutex state_mtx;
inno_ligo::sensor_status state_pub;
bool flg_exit = false;
deque<PointCloudXYZI::Ptr> lidar_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
deque<sensor_msgs::NavSatFix::ConstPtr> gps_buffer;
inno_ligo::sensor_measures measures_group;
// threads
auto lo_ctr = new ThreadController();
auto lio_ctr = new ThreadController();
auto gio_ctr = new ThreadController();
auto ligo_ctr = new ThreadController();

void lidar_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg_in) {
  state_mtx.lock();
  if (!SBits->sensor_flow.test(0)) {
    SBits->sensor_flow[0] = 1;
  }
  state_mtx.unlock();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) {
  // state_mtx.lock();
  // if (!SBits->sensor_flow.test(1)) {
  //   SBits->sensor_flow[1] = 1;
  // }
  // state_mtx.unlock();
  // std::cout << "in callback. " << std::endl;
}

void gps_cbk(const sensor_msgs::NavSatFix::ConstPtr &msg_in) {
  state_mtx.lock();
  if (!SBits->sensor_flow.test(2)) {
    SBits->sensor_flow[2] = 1;
  }
  state_mtx.unlock();
}

void PublishState(const ros::Publisher &state_puber) {
  state_pub.header.stamp = ros::Time().fromSec(1);
  state_pub.lidar = SBits->sensor_flow[0];
  state_pub.imu = SBits->sensor_flow[1];
  state_pub.gps = SBits->sensor_flow[2];

  state_puber.publish(state_pub);
}

void SigHandle(int sig) {
  flg_exit = true;
  ROS_WARN("catch sig %d", sig);
  // sig_buffer
  //     .notify_all();  //  会唤醒所有等待队列中阻塞的线程
  //                     //
  //                     线程被唤醒后，会通过轮询方式获得锁，获得锁前也一直处理运行状态，不会被再次阻塞。
}

TestLoth test_loth;

void test_lll() {
  while (1) {
    int count_ = 0;
    std::cout << "count_: " << count_++ << std::endl;
  }
}

// bool sync_packages(MeasureGroup &meas) {
//   if (imu_buffer.empty() || gps_buffer.empty()) {
//     return false;
//   }
//   double gps_time = 0.0;
//   /*** push gps data, and pop from imu buffer ***/
//   meas.gps.clear();

//   while ((!gps_buffer.empty())) {
//     gps_time = gps_buffer.front()[0];
//     meas.gps.push_back(gps_buffer.front());
//     gps_buffer.pop_front();
//   }
//   gps_curr_time = gps_time;
//   /*** push imu data, and pop from imu buffer ***/
//   double imu_time = imu_buffer.front()->header.stamp.toSec();
//   // std::cout << " gps_time: " << gps_time << "   imu time: " << imu_time
//   //           << "  imu buffer size: " << imu_buffer.size() << std::endl;
//   meas.imu.clear();
//   while ((!imu_buffer.empty()) && (imu_time < gps_time)) {
//     imu_time = imu_buffer.front()->header.stamp.toSec();
//     if (imu_time > gps_time) break;
//     // if (gps_time - imu_time > 0.01) {
//     //   debug_count++;
//     //   imu_buffer.pop_front();
//     // } else {
//     sensor_msgs::Imu::Ptr temp_imu(new sensor_msgs::Imu());

//     NED2ENU(imu_buffer.front(), temp_imu);

//     meas.imu.push_back(temp_imu);
//     imu_buffer.pop_front();
//     // }
//   }
//   std::cout << " meas.imu.size: " << meas.imu.size()
//             << "    meas.gps.size: " << meas.gps.size() << std::endl;
//   time_buffer.pop_front();
//   return true;
// }

int main(int argc, char **argv) {
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;

  nh.param<std::string>("common/lidar_topic", lidar_topic, "/lidar");
  nh.param<std::string>("common/imu_topic", imu_topic, "/imu");
  nh.param<std::string>("common/gps_topic", gps_topic, "/gps");

  // ros::Subscriber sub_lidar = nh.subscribe(gps_topic, 200000, lidar_cbk);
  ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
  // ros::Subscriber sub_gps = nh.subscribe(gps_topic, 200000, gps_cbk);

  // state monitor

  // ros::Publisher pub_state =
  //     nh.advertise<inno_ligo::sensor_status>("/state_flow", 100000);

  ros::Rate rate(5000);
  bool status = ros::ok();
  signal(SIGINT, SigHandle);
  while (status) {
    if (flg_exit) break;
    ros::spinOnce();
    std::thread test_th(test_lll);
    test_th.join();

    // PublishState(pub_state);
    status = ros::ok();
    rate.sleep();
  }
  // std::thread test_th(test_lll);
  // test_th.join();
  ros::spin();
  // std::thread(LOProc, SM, lo_ctr, meas);
  // std::thread(LIOProc, SM, lio_ctr, meas);
  // std::thread(LIGOProc, SM, ligo_ctr, meas);

  return 0;
}
// TODO：在主节点监控传感器状态，根据status调用method
// TODO：将LO LIO GIO 封装成一个thread
// TODO:

/*
//*put in measureGroup


subscribe(state, callback(state))

callback(state){
  thread th_1(LO)
}

/*/