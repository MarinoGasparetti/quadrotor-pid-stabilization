#ifndef DISTURBANCE_OBSERVER_HPP
#define DISTURBANCE_OBSERVER_HPP

#include <cmath>
#include <algorithm>

/**
 * @brief Modulo per la stima e compensazione dei disturbi (Vento/Inerzia).
 * Implementa la logica della derivata dell'errore marginale rispetto al comando utente.
 */
class DisturbanceObserver {
public:
    struct State {
        double user_pitch_intent;    // Comando utente (Target)
        double actual_acceleration;  // Accelerazione reale (IMU)
        double estimated_mass;       // Massa stimata per calcolo inerziale
    };

    DisturbanceObserver(double sensitivity) 
        : sensitivity_(sensitivity), last_marginal_error_(0), initialized_(false) {}

    /**
     * @brief Calcola l'offset di correzione basato sulla mutazione della traiettoria.
     * @return Offset da sommare al setpoint del PID.
     */
    double computeCorrection(const State& state, double dt) {
        if (dt < 1e-6) return 0;

        // 1. Calcoliamo l'accelerazione attesa dal comando utente (semplificato)
        double expected_accel = state.user_pitch_intent * 0.1;

        // 2. Errore Marginale: Quanto il mondo reale devia dall'intento
        double marginal_error = expected_accel - state.actual_acceleration;

        // 3. Calcolo derivata con protezione anti-spike all'avvio
        double error_mutation = 0;
        if (initialized_) {
            error_mutation = (marginal_error - last_marginal_error_) / dt;
        } else {
            initialized_ = true;
        }

        // 4. Correzione Predittiva con Clamping per sicurezza fisica
        double raw_correction = (marginal_error * 0.5 + error_mutation * sensitivity_);
        double correction = std::max(-20.0, std::min(raw_correction, 20.0));

        last_marginal_error_ = marginal_error;

        return correction;
    }

    void reset() {
        initialized_ = false;
        last_marginal_error_ = 0;
    }

private:
    double sensitivity_;
    double last_marginal_error_;
    bool initialized_;
};

#endif // DISTURBANCE_OBSERVER_HPP
