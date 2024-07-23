#include <condition_variable>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

#include "sensor_flow.hpp"

class ThreadController {
 public:
  ThreadController() : m_flag(false) {}
  void wait() {
    std::unique_lock<std::mutex> lock(m_mtx);
    m_cv.wait(lock, [=] { return m_flag; });
    m_flag = false;
  }
  void wake() {
    std::unique_lock<std::mutex> lock(m_mtx);
    m_flag = true;
    m_cv.notify_one();
  }

 private:
  std::mutex m_mtx;
  std::condition_variable m_cv;
  bool m_flag;
};

class ControlManager {
 public:
  ControlManager() {}
  ~ControlManager() {
    for (auto ctrl : m_CtrlVec) delete ctrl;
    m_CtrlVec.clear();
  }
  ThreadController* createCtrl() {
    auto ctrl = new ThreadController();
    m_mutex.lock();
    m_CtrlVec.push_back(std::move(ctrl));
    m_mutex.unlock();
    return ctrl;
  }
  std::vector<ThreadController*>& getAllCtrl() { return m_CtrlVec; }

 private:
  std::vector<ThreadController*> m_CtrlVec;
  std::mutex m_mutex;
};

class TestLoth {
 public:
  TestLoth() { count = 0; };
  void proc() {
    count++;
    std::cout << "count: " << count << std::endl;
  }
  int count;
};