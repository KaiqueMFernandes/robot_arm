#include <chrono>
#include <mutex>
#include <conditional_variable>
#include <queue>
#include <iostream>
#include <fstream>
#include <thread>
#include <array>

#include "state.hpp"

using namespace robot_arm;

// Thread-safe queue
template <typename T>
class SafeQueue {
public:
    void push(const T& value) {
        std::lock_guard<std::mutex> lock(mtx_);
        q_.push(value);
        cv_.notify_one();
    }

    std::optional<T> pop() {
        std::unique_lock<std::mutex> lock(mtx_);
        if (q_.empty())
            return std::nullopt;
        T val = q_.front();
        q_.pop();
        return val;
    }

private:
    std::queue<T> q_;
    std::mutex mtx_;
    std::condition_variable cv_;
};

void main() {
    std::thread imu(imuSensor);

    std::this_thread::sleep_for(std::chrono::seconds(5));

    imu.join();

    std::cout << "Simulation finished.\n";
    return 0;
}