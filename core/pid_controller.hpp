// core/pid_controller.hpp [sha:af33d58483b4855f4872ba4daf3211768b5f0d47]

#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <algorithm>

/**
 * @brief PID Controller evoluto con supporto a profili concorrenti per Volo e Statico (Hovering).
 */
class PIDController {
public:
    enum class FlightMode {
        STATIC,   // Ottimizzato per hovering e stabilità massima (meno vibrazioni)
        DYNAMIC   // Ottimizzato per reattività e manovre rapide
    };

    struct Params {
        double kp;
        double ki;
        double kd;
        double min_output;
        double max_output;
    };

    struct MultiProfileParams {
        Params static_profile;  // Parametri per hovering
        Params dynamic_profile; // Parametri per volo acrobatico/rapido
    };

    PIDController(const MultiProfileParams& profiles) 
        : profiles_(profiles), mode_(FlightMode::STATIC), integral_(0), last_error_(0) {}

    /**
     * @brief Cambia il profilo di volo in tempo reale (Gain Scheduling).
     */
    void setFlightMode(FlightMode mode) {
        if (mode_ != mode) {
            mode_ = mode;
            // Reset parziale per evitare picchi durante la transizione
            integral_ *= 0.5; 
        }
    }

    /**
     * @brief Calcolo PID basato sul profilo attivo.
     */
    double compute(double setpoint, double current_value, double dt) {
        const Params& active = (mode_ == FlightMode::STATIC) ? profiles_.static_profile : profiles_.dynamic_profile;
        
        double error = setpoint - current_value;

        // Proporzionale
        double p_out = active.kp * error;

        // Integrale con anti-windup semplice
        integral_ += error * dt;
        double i_out = active.ki * integral_;

        // Derivata
        double derivative = 0;
        if (dt > 1e-6) {
            derivative = (error - last_error_) / dt;
        }
        double d_out = active.kd * derivative;

        double output = p_out + i_out + d_out;

        // Limiti output
        output = std::max(active.min_output, std::min(output, active.max_output));

        last_error_ = error;
        return output;
    }

    void reset() {
        integral_ = 0;
        last_error_ = 0;
    }

private:
    MultiProfileParams profiles_;
    FlightMode mode_;
    double integral_;
    double last_error_;
};

#endif // PID_CONTROLLER_HPP
