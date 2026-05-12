#ifndef DISTURBANCE_OBSERVER_HPP
#define DISTURBANCE_OBSERVER_HPP

#include <cmath>

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
        : sensitivity_(sensitivity), last_marginal_error_(0) {}

    /**
     * @brief Calcola l'offset di correzione basato sulla mutazione della traiettoria.
     * @return Offset da sommare al setpoint del PID.
     */
    double computeCorrection(const State& state, double dt) {
        if (dt < 1e-6) return 0;

        // 1. Calcoliamo l'accelerazione attesa dal comando utente (semplificato)
        // In un sistema reale, qui useremmo la proiezione della spinta motori.
        double expected_accel = state.user_pitch_intent * 0.1; // Coeff. di traslazione

        // 2. Errore Marginale: Quanto il mondo reale devia dall'intento
        double marginal_error = expected_accel - state.actual_acceleration;

        // 3. Derivata della curva di errore marginale
        // Rappresenta la "tendenza" del disturbo (es. la folata che sta crescendo)
        double error_mutation = (marginal_error - last_marginal_error_) / dt;

        // 4. Correzione Predittiva
        // Tendiamo verso il pitch utente compensando la mutazione prima che diventi deriva.
        double correction = (marginal_error * 0.5 + error_mutation * sensitivity_);

        last_marginal_error_ = marginal_error;

        return correction;
    }

private:
    double sensitivity_;
    double last_marginal_error_;
};

#endif // DISTURBANCE_OBSERVER_HPP
