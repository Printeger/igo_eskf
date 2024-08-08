#include <common_lib.h>
// #include <proj_api.h>
#include <common_msgs/LinktrackNodeframe2.h>
#include <sensor_msgs/NavSatFix.h>

#include <GeographicLib/LocalCartesian.hpp>
#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

#include "use-ikfom.hpp"

// class PJTransformer
// {
// public:
//   explicit PJTransformer(double lon_0, double lat_0) {
//     // init projPJ
//     std::stringstream stream;
//     // stream << "+proj=utm +zone=" << zone_id << " +ellps=WGS84" <<
//     std::endl;
//     // stream << "+proj=tmerc +k=0.9996 +lon_0=" << lon_0 << "e +ellps=WGS84"
//     //        << std::endl;
//     stream << "+proj=tmerc +k=0.9996 +lon_0=" << lon_0 << " +lat_0= " <<
//     lat_0
//            << "e +ellps=WGS84" << std::endl;
//     pj_utm_ = pj_init_plus(stream.str().c_str());
//     // pj_utm_ = pj_init_plus_ctx();
//     if (pj_utm_ == nullptr) {
//       std::cout << "proj4 init failed!" << stream.str() << std::endl;
//       return;
//     }
//     pj_latlong_ = pj_init_plus("+proj=latlong +ellps=WGS84");
//     if (pj_latlong_ == nullptr) {
//       std::cout << "proj4 pj_latlong init failed!";
//       return;
//     }
//     std::cout << "proj4 init success" << std::endl;
//   }

//   ~PJTransformer() {
//     if (pj_latlong_) {
//       pj_free(pj_latlong_);
//       pj_latlong_ = nullptr;
//     }
//     if (pj_utm_) {
//       pj_free(pj_utm_);
//       pj_utm_ = nullptr;
//     }
//   }

//   int LatlonToUtm(int64_t point_count, int point_offset, double *x, double
//   *y,
//                   double *z) {
//     if (!pj_latlong_ || !pj_utm_) {
//       std::cout << "pj_latlong_:" << pj_latlong_ << "pj_utm_:" << pj_utm_
//                 << std::endl;
//       return -1;
//     }
//     return pj_transform(pj_latlong_, pj_utm_, point_count, point_offset, x,
//     y,
//                         z);
//   }

//  private:
//   projPJ pj_latlong_;
//   projPJ pj_utm_;
// };

class GPSProcess {
 public:
  GPSProcess();
  ~GPSProcess();

  // gps 2 utm
  // set_gps_cov()
  void Initialize(double lon_init, double lat_init);
  void Initialize(const double lon_init, const double lat_init,
                  const double alti_init);
  void Reset();
  void set_gps_cov(const V3D &scaler);
  void set_extrinsic(const MD(4, 4) & T);
  void set_extrinsic(const V3D &trans);
  void set_extrinsic(const V3D &trans, const M3D &rot);
  void LLA2UTM(const V3D &lla_, V3D &utm_);
  Eigen::Vector3d ECEF2ENU(const Eigen::Vector3d &vel_ecef, double lat_ref,
                           double lon_ref);
  void Process(const sensor_msgs::NavSatFix::ConstPtr &msg, GPSGroup &gps_out);
  void Process(const gnss_comm::GnssPVTSolnMsg::ConstPtr &msg,
               GPSGroup &gps_out);
  void uwb_process(const common_msgs::LinktrackNodeframe2::ConstPtr &msg,
                   GPSGroup &gps_out);

  GeographicLib::LocalCartesian geo_converter;
  sensor_msgs::NavSatFixPtr last_gps_;
  std::vector<V3D> GPS_position;
  double timestamp;
  V3D LLA;  // lon, lat, alt
  V3D UTM;
  V3D cov_gps;
  M3D GPS_R_wrt_IMU;  // GPS to IMU extrinsic
  V3D GPS_T_wrt_IMU;
  // std::shared_ptr<PJTransformer> proj_ = nullptr;
  // PJTransformer proj_;
  bool proj_init = false;
  double lon_0 = 120.0;
  double lat_0 = 30.0;
  double alti_0 = 0.0;
};

GPSProcess::GPSProcess() {
  LLA = V3D(0, 0, 0);
  UTM = V3D(0, 0, 0);
  cov_gps = V3D(0.1, 0.1, 0.1);
  GPS_R_wrt_IMU = Eye3d;
  GPS_T_wrt_IMU = Zero3d;
  last_gps_.reset(new sensor_msgs::NavSatFix());
}

GPSProcess::~GPSProcess() {}

void GPSProcess::Reset() {
  LLA = V3D(0, 0, 0);
  UTM = V3D(0, 0, 0);

  GPS_position.clear();
  last_gps_.reset(new sensor_msgs::NavSatFix());
}

void GPSProcess::set_extrinsic(const MD(4, 4) & T) {
  GPS_T_wrt_IMU = T.block<3, 1>(0, 3);
  GPS_R_wrt_IMU = T.block<3, 3>(0, 0);
}

void GPSProcess::set_extrinsic(const V3D &trans) {
  GPS_T_wrt_IMU = trans;
  GPS_R_wrt_IMU.setIdentity();
}

void GPSProcess::set_extrinsic(const V3D &trans, const M3D &rot) {
  GPS_T_wrt_IMU = trans;
  GPS_R_wrt_IMU = rot;
}

void GPSProcess::set_gps_cov(const V3D &scaler) { cov_gps = scaler; }

Eigen::Vector3d GPSProcess::ECEF2ENU(const Eigen::Vector3d &vel_ecef,
                                     double lat_ref, double lon_ref) {
  // 计算旋转矩阵的各个元素
  double sin_lat = std::sin(lat_ref);
  double cos_lat = std::cos(lat_ref);
  double sin_lon = std::sin(lon_ref);
  double cos_lon = std::cos(lon_ref);

  // 旋转矩阵
  Eigen::Matrix3d R;
  R << -sin_lon, cos_lon, 0, -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat,
      cos_lat * cos_lon, cos_lat * sin_lon, sin_lat;

  // 转换速度向量
  Eigen::Vector3d vel_enu;
  vel_enu = R * vel_ecef;
  // vel_enu.x = R[0][0] * vel_ecef.x + R[0][1] * vel_ecef.y + R[0][2] *
  // vel_ecef.z; vel_enu.y = R[1][0] * vel_ecef.x + R[1][1] * vel_ecef.y +
  // R[1][2] * vel_ecef.z; vel_enu.z = R[2][0] * vel_ecef.x + R[2][1] *
  // vel_ecef.y + R[2][2] * vel_ecef.z;

  return vel_enu;
}

void GPSProcess::LLA2UTM(const V3D &lla_, V3D &utm_enu) {
  geo_converter.Forward(lla_[0], lla_[1], lla_[2], utm_enu[0], utm_enu[1],
                        utm_enu[2]);
  // double lon_, lat_, alt_;
  // lat_ = lla_[0] / 180 * M_PI;
  // lon_ = lla_[1] / 180 * M_PI;
  // alt_ = lla_[2];
  // proj_->LatlonToUtm(1, 1, &lon_, &lat_, &alt_);
  // // proj_->LatlonToUtm(1, 1, &lat_, &lon_, &alt_);

  // utm_enu[0] = lon_;
  // utm_enu[1] = lat_;
  // utm_enu[2] = alt_;
}
//  initialize proj zero point as the first coming GPS position
void GPSProcess::Initialize(double lon_init, double lat_init) {
  if (!proj_init) {
    lon_0 = lon_init;
    lat_0 = lat_init;
    proj_init = true;
  }
  // 经纬度转ENU
  // // 初始化UTM原点坐标，这里 lat/lon 顺序不影响
  // proj_ = std::make_shared<PJTransformer>(lon_0, lat_0);
  // std::cout << "=====UTM initialize DONE=====" << std::endl;
}
// TODO:
void GPSProcess::Initialize(const double lon_init, const double lat_init,
                            const double alti_init) {
  if (!proj_init) {
    lon_0 = lon_init;
    lat_0 = lat_init;
    alti_0 = alti_init;
    // TODO: DEBUG
    geo_converter.Reset(lat_init, lon_init, alti_init);

    proj_init = true;
  }
}

void GPSProcess::Process(const sensor_msgs::NavSatFix::ConstPtr &msg,
                         GPSGroup &gps_out) {
  timestamp = msg->header.stamp.toSec();
  LLA[0] = msg->latitude;
  LLA[1] = msg->longitude;
  LLA[2] = msg->altitude;
  LLA2UTM(LLA, UTM);
  gps_out.timestamp = timestamp;
  gps_out.LLA = LLA;
  gps_out.UTM = UTM;
  // gps_out = V4D(timestamp, UTM[0], UTM[1], UTM[2]);
}

void GPSProcess::Process(const gnss_comm::GnssPVTSolnMsg::ConstPtr &msg,
                         GPSGroup &gps_out) {
  timestamp = msg->vel_acc;
  ROS_INFO("timestamp: %f", timestamp);
  LLA[0] = msg->latitude;
  LLA[1] = msg->longitude;
  LLA[2] = msg->altitude;
  LLA2UTM(LLA, UTM);
  gps_out.timestamp = timestamp;
  gps_out.LLA = LLA;
  gps_out.UTM = UTM;
  Eigen::Vector3d vel_ecef(msg->vel_e, msg->vel_n, msg->vel_d);
  gps_out.velocity = ECEF2ENU(vel_ecef, LLA[0], LLA[1]);
  // gps_out.velocity = V3D(msg->vel_e, msg->vel_n, -msg->vel_d);
  // gps_out = V4D(timestamp, UTM[0], UTM[1], UTM[2]);
}

void GPSProcess::uwb_process(
    const common_msgs::LinktrackNodeframe2::ConstPtr &msg, GPSGroup &gps_out) {
  timestamp = msg->local_time * 1e-6;
  LLA[0] = msg->pos_3d.at(0) - lon_0;
  LLA[1] = msg->pos_3d.at(1) - lat_0;
  LLA[2] = msg->pos_3d.at(2) - alti_0;
  UTM[0] = msg->pos_3d.at(0) - lon_0;
  UTM[1] = msg->pos_3d.at(1) - lat_0;
  UTM[2] = msg->pos_3d.at(2) - alti_0;
  gps_out.timestamp = timestamp;
  gps_out.LLA = LLA;
  gps_out.UTM = UTM;
}