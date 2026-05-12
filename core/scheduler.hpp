// core/scheduler.hpp
#ifndef SCHEDULER_HPP
#define SCHEDULER_HPP

#include <chrono>
#include <thread>
#include <functional>
#include <atomic>
#include <vector>
#include "pid_controller.hpp"
#include "disturbance_observer.hpp"

/**
 * @brief Scheduler asincrono per la gestione concorrente.
 * Connette il DisturbanceObserver al PID Controller tramite i task Inner e Outer.
 */
class DroneScheduler {
public:
    DroneScheduler(PIDController& pid, DisturbanceObserver& observer) 
        : pid_(pid), observer_(observer), running_(false), user_pitch_target_(0.0), current_imu_pitch_(0.0) {}

    ~DroneScheduler() { stop(); }

    void start() {
        running_ = true;
        // Task 1: Inner Loop - Alta frequenza (400Hz)
        // Si occupa solo della stabilizzazione immediata
        scheduler_threads_.emplace_back([this]() {
            while (running_) {
                double dt = 0.0025; // 400Hz
                // Applica il comando corretto dal vento
                double output = pid_.compute(corrected_target_.load(), current_imu_pitch_.load(), dt);
                // Qui l'output verrebbe inviato ai motori
                std::this_thread::sleep_for(std::chrono::microseconds(2500));
            }
        });

        // Task 2: Outer Loop - Media frequenza (50Hz)
        // Calcola la compensazione del vento e aggiorna il target per l'Inner Loop
        scheduler_threads_.emplace_back([this]() {
            while (running_) {
                double dt = 0.02; // 50Hz
                
                // Calcola la correzione basata sulla "derivata dell'errore marginale"
                double wind_offset = observer_.estimateCorrection(user_pitch_target_.load(), current_imu_pitch_.load(), dt);
                
                // Il target reale per i motori è il comando utente + la compensazione vento
                corrected_target_.store(user_pitch_target_.load() + wind_offset);
                
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        });
    }

    void stop() {
        running_ = false;
        for (auto& t : scheduler_threads_) {
            if (t.joinable()) t.join();
        }
    }

    // Metodi per aggiornare gli input dall'esterno (es. radiocomando e sensori)
    void setUserInput(double pitch) { user_pitch_target_.store(pitch); }
    void updateIMU(double pitch) { current_imu_pitch_.store(pitch); }

private:
    PIDController& pid_;
    DisturbanceObserver& observer_;
    std::atomic<bool> running_;
    std::vector<std::thread> scheduler_threads_;

    // Stati condivisi tra i thread (thread-safe)
    std::atomic<double> user_pitch_target_;
    std::atomic<double> current_imu_pitch_;
    std::atomic<double> corrected_target_;
};

#endif // SCHEDULER_HPP
