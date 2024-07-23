#include <bitset>
#include <string>

#include "../include/common_lib.h"

/**
 * @brief
    Sensor: 1) lidar 2) imu 3) gps
    1: availble   0: not available

    current localization method: 1) Lidar_odometry   2) LIO   3) LIGO   4) GIO
    1: in use   0: unused
 *
 */

class StateMonitor {
 public:
  StateMonitor();
  ~StateMonitor(){};

  bool UpdateStates(std::bitset<3> &curr_sen_flow,
                    std::bitset<3> &curr_meth_flow);

  std::bitset<3> sensor_flow;
  std::bitset<3> method_flow;
};

StateMonitor::StateMonitor() {
  sensor_flow.reset();
  method_flow.reset();
}

bool StateMonitor::UpdateStates(std::bitset<3> &curr_sen_flow,
                                std::bitset<3> &curr_meth_flow) {
  sensor_flow = curr_sen_flow;
  method_flow = curr_meth_flow;

  return true;
}

// class LO {
//  public:
//   LOProc(){};
//   ~LOProc(){};

//   void LOProcess(StateMonitor &curr_state MeasureGroup &meas) {
//     // init --> LO 000 --> 100
//     if (!last_state.sensor_flow.test(0) && curr_state.sensor_flow.test(0) &&
//         !curr_state.sensor_flow.test(1)) {
//       // ===run LO===
//     }
//     // L --> L  100 --> 100
//     else if (last_state.sensor_flow.test(0) && curr_state.sensor_flow.test(0)
//     &&
//              !curr_state.sensor_flow.test(1)) {
//       // ===continue LO===
//     }
//     // L --> LI  100 --> 110
//     else if (last_state.sensor_flow.test(0) && curr_state.sensor_flow.test(0)
//     &&
//              curr_state.sensor_flow.test(1)) {
//       // ===keep running LO for xxx frame, then hold===
//     }
//     // 100 / 110 --> 111
//     else if (last_state.sensor_flow.test(0) &&
//              curr_state.sensor_flow.count() == 3)

//       else {
//         // ===report false===
//       }
//   }
//   StateMonitor last_state;
// };

// class LIO {
//  public:
//   LIOProc(){};
//   ~LIOProc(){};

//   void LIOProcess(StateMonitor &curr_state MeasureGroup &meas) {
//     // init --> LIO  000 --> 011
//     if (!last_state.sensor_flow.test(0) && curr_state.sensor_flow.test(0) &&
//         curr_state.sensor_flow.test(1)) {
//       // ===run LO===
//     }
//     // L --> L
//     else if (last_state.sensor_flow.test(0) && curr_state.sensor_flow.test(0)
//     &&
//              !curr_state.sensor_flow.test(1)) {
//       // ===continue LO===
//     }
//     // L --> LI
//     else if (last_state.sensor_flow.test(0) && curr_state.sensor_flow.test(0)
//     &&
//              curr_state.sensor_flow.test(1)) {
//       // ===keep running LO for xxx frame, then hold===
//     } else {
//       // ===report false===
//     }
//   }
//   StateMonitor last_state;
// };
