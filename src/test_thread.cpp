#include <condition_variable>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

#define WAY 1  // 设置唤醒线程方式

#if WAY == 1  // 方式1，每个线程拥有独立的锁和条件变量
class ThreadCtrl {
 public:
  ThreadCtrl() : m_flag(false) {}
  void wait() {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_cv.wait(lock, [=] { return m_flag; });
    m_flag = false;
  }
  void wake() {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_flag = true;
    m_cv.notify_one();
  }

 private:
  std::mutex m_mutex;
  std::condition_variable m_cv;
  bool m_flag;
};

class ThreadManage {
 public:
  ThreadManage() {}
  ~ThreadManage() {
    for (auto ctrl : m_CtrlVec) delete ctrl;
    m_CtrlVec.clear();
  }
  ThreadCtrl* createCtrl() {
    auto ctrl = new ThreadCtrl();
    m_mutex.lock();
    m_CtrlVec.push_back(std::move(ctrl));
    m_mutex.unlock();
    return ctrl;
  }
  std::vector<ThreadCtrl*>& getAllCtrl() { return m_CtrlVec; }

 private:
  std::vector<ThreadCtrl*> m_CtrlVec;
  std::mutex m_mutex;
};
#elif WAY == 2  // 方式2，所有线程共用同一锁，但每个线程拥有独立的条件变量
class ThreadCtrl {
 public:
  ThreadCtrl(std::mutex* mutex) : m_mutex(mutex), m_flag(false) {}
  void wait() {
    std::unique_lock<std::mutex> lock(*m_mutex);
    m_cv.wait(lock, [=] { return m_flag; });
    m_flag = false;
  }
  void wake() {
    std::unique_lock<std::mutex> lock(*m_mutex);
    m_flag = true;
    m_cv.notify_one();
  }

 private:
  std::mutex* m_mutex;
  std::condition_variable m_cv;
  bool m_flag;
};

class ThreadManage {
 public:
  ThreadManage() {}
  ~ThreadManage() {
    for (auto ctrl : m_CtrlVec) delete ctrl;
    m_CtrlVec.clear();
  }
  ThreadCtrl* createCtrl() {
    auto ctrl = new ThreadCtrl(&m_wakeMutex);
    m_mutex.lock();
    m_CtrlVec.push_back(std::move(ctrl));
    m_mutex.unlock();
    return ctrl;
  }
  std::vector<ThreadCtrl*>& getAllCtrl() { return m_CtrlVec; }

 private:
  std::vector<ThreadCtrl*> m_CtrlVec;
  std::mutex m_mutex;
  std::mutex m_wakeMutex;
};
#elif WAY == 3  // 方式3，所有的线程共用同一锁和同一条件变量
class ThreadCtrl {
 public:
  ThreadCtrl(std::mutex* mutex, std::condition_variable* cv, bool* flag)
      : m_mutex(mutex), m_cv(cv), m_flag(flag) {}
  void wait() {
    std::unique_lock<std::mutex> lock(*m_mutex);
    m_cv->wait(lock, [=] { return *m_flag; });
    *m_flag = false;
  }
  void wake() {
    std::unique_lock<std::mutex> lock(*m_mutex);
    *m_flag = true;
    m_cv->notify_all();  // 所有线程共用条件变量，所以必须通知所有等待的线程
  }

 private:
  std::mutex* m_mutex;
  std::condition_variable* m_cv;
  bool* m_flag;
};

class ThreadManage {
 public:
  ThreadManage() {}
  ~ThreadManage() {
    for (auto ctrl : m_CtrlVec) delete ctrl;
    m_CtrlVec.clear();
  }
  ThreadCtrl* createCtrl() {
    auto flag = new bool(false);
    auto ctrl = new ThreadCtrl(&m_wakeMutex, &m_cv, flag);
    m_mutex.lock();
    m_flagVec.push_back(std::move(flag));
    m_CtrlVec.push_back(std::move(ctrl));
    m_mutex.unlock();
    return ctrl;
  }
  std::vector<ThreadCtrl*>& getAllCtrl() { return m_CtrlVec; }

 private:
  std::vector<ThreadCtrl*> m_CtrlVec;
  std::mutex m_mutex;
  std::mutex m_wakeMutex;
  std::condition_variable m_cv;
  std::vector<bool*> m_flagVec;
};
#endif

// 线程ID转为数字
long long threadIdToNumber(const std::thread::id& id) {
  std::stringstream oss;
  oss << id;
  return std::stoll(oss.str());
}

// 测试线程内执行的函数
void fun(ThreadCtrl* ctrl) {
  while (true) {
    ctrl->wait();  // 睡眠
#if 0
        auto tid = std::this_thread::get_id();
        auto lid = threadIdToNumber(tid);
        printf("Thread ID: %lld\n", lid);
#endif
  }
}

// 负责唤醒其它线程
void wakeFun(ThreadManage* manage) {
  std::vector<ThreadCtrl*> allCtrl = manage->getAllCtrl();
  const int num = allCtrl.size();
  int id = num - 1;
  while (true) {
    allCtrl[id]->wake();  // 唤醒
    id--;
    if (id < 0) id = num - 1;
    // 等待1ms，尽可能确保所有线程处于等待状态
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

int main() {
  // 测试线程个数
  constexpr const int count = 3000;
  // 创建线程管理，管理线程唤醒
  auto manage = new ThreadManage;
  // 创建测试线程
  std::vector<std::thread> threadVec(count);
  for (int i = 0; i < count; i++) {
    auto ctrl = manage->createCtrl();
    threadVec[i] = std::thread(fun, ctrl);
  }
  std::thread wakeThread(wakeFun, manage);
  // 等待子线程
  for (int i = 0; i < count; i++) threadVec[i].join();
  wakeThread.join();
  // 释放资源
  delete manage;
}
