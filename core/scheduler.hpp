#ifndef SCHEDULER_HPP
#define SCHEDULER_HPP

#include <chrono>
#include <thread>
#include <functional>
#include <atomic>
#include <vector>

/**
 * @brief Scheduler asincrono per la gestione concorrente dei task del drone.
 * Separa il loop di stabilizzazione ad alta frequenza (Inner) 
 * dal loop di navigazione/statico a frequenza ridotta (Outer).
 */
class DroneScheduler {
public:
    struct Task {
        std::function<void()> func;
        double frequency_hz;
        std::chrono::steady_clock::time_point last_run;
    };

    DroneScheduler() : running_(false) {}

    ~DroneScheduler() {
        stop();
    }

    void addTask(std::function<void()> func, double frequency_hz) {
        tasks_.push_back({func, frequency_hz, std::chrono::steady_clock::now()});
    }

    void start() {
        running_ = true;
        scheduler_thread_ = std::thread(&DroneScheduler::run, this);
    }

    void stop() {
        running_ = false;
        if (scheduler_thread_.joinable()) {
            scheduler_thread_.join();
        }
    }

private:
    void run() {
        while (running_) {
            auto now = std::chrono::steady_clock::now();
            
            for (auto& task : tasks_) {
                auto interval = std::chrono::nanoseconds(static_cast<long long>(1e9 / task.frequency_hz));
                if (now - task.last_run >= interval) {
                    task.func();
                    task.last_run = now;
                }
            }
            
            // Piccola pausa per non saturare il core se non ci sono task pronti
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

    std::atomic<bool> running_;
    std::vector<Task> tasks_;
    std::thread scheduler_thread_;
};

#endif // SCHEDULER_HPP
