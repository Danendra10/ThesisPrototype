#ifndef TIMER_HPP_
#define TIMER_HPP_

#include <iostream>
#include <chrono>
#include <thread>
#include <functional>
#include <atomic>
#include <condition_variable>
#include <mutex>

class Timer
{
public:
    using TimerCallback = std::function<void()>;

    // Create a timer that ticks every interval milliseconds
    // So if you want to put your desired frequency in here
    Timer(const TimerCallback &callback, float interval)
        : callback_(callback),
          interval_(interval),
          running_(false) {}

    ~Timer()
    {
        stop();
    }

    void start()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (running_)
                return; // Prevent double-starting
            running_ = true;
        }

        worker_thread_ = std::thread([this]
                                     {
            while (running_) {
                auto next_tick = std::chrono::steady_clock::now() + std::chrono::duration<float, std::milli>(interval_);
                callback_();
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait_until(lock, next_tick, [this] { return !running_; });
            } });
    }

    void stop()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!running_)
                return;
            running_ = false;
        }
        cv_.notify_one();
        if (worker_thread_.joinable())
        {
            worker_thread_.join();
        }
    }

    bool isRunning() const
    {
        return running_.load();
    }

private:
    TimerCallback callback_;
    float interval_; // Interval between ticks in milliseconds
    std::atomic<bool> running_;
    std::thread worker_thread_;
    std::condition_variable cv_;
    std::mutex mutex_;
};

#endif