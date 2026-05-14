import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

class PIDController:
    def __init__(self, kp_static, ki_static, kd_static, kp_dyn, ki_dyn, kd_dyn):
        self.params = {
            'STATIC': {'kp': kp_static, 'ki': ki_static, 'kd': kd_static},
            'DYNAMIC': {'kp': kp_dyn, 'ki': ki_dyn, 'kd': kd_dyn}
        }
        self.mode = 'STATIC'
        self.integral = 0
        self.last_error = 0

    def set_mode(self, mode):
        if self.mode != mode:
            self.mode = mode
            self.integral *= 0.5  # Reset parziale come in core/pid_controller.hpp

    def compute(self, setpoint, measurement, dt):
        p = self.params[self.mode]
        error = setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        
        output = (p['kp'] * error) + (p['ki'] * self.integral) + (p['kd'] * derivative)
        self.last_error = error
        return np.clip(output, -100, 100)

class DisturbanceObserver:
    def __init__(self, sensitivity):
        self.sensitivity = sensitivity
        self.last_marginal_error = 0

    def compute_correction(self, user_pitch, actual_accel, dt):
        if dt < 1e-6: return 0
        expected_accel = user_pitch * 0.1
        marginal_error = expected_accel - actual_accel
        error_mutation = (marginal_error - self.last_marginal_error) / dt
        
        # Logica fedele a core/disturbance_observer.hpp
        correction = (marginal_error * 0.5 + error_mutation * self.sensitivity)
        self.last_marginal_error = marginal_error
        
        # Clamping di sicurezza (anti-overflow locale)
        return np.clip(correction, -10, 10)

def run_advanced_sim():
    dt = 0.01
    time = np.arange(0, 10, dt)
    user_pitch = 15.0
    
    # Init componenti con logica reale
    pid = PIDController(1.5, 0.2, 0.5, 2.5, 0.5, 0.8)
    dob = DisturbanceObserver(sensitivity=0.6)
    
    pitch = 0.0
    velocity = 0.0
    accel = 0.0
    
    history = {'pitch': [], 'correction': [], 'mode': [], 'wind': []}

    for t in time:
        # 1. Switch di modo (Gain Scheduling)
        if t > 5.0:
            pid.set_mode('DYNAMIC')
        
        # 2. Profilo Vento (Disturbo)
        wind = -12.0 if 3.0 <= t <= 6.0 else 0.0
        
        # 3. DOB Correction
        correction = dob.compute_correction(user_pitch, accel, dt)
        
        # 4. PID Control
        control_output = pid.compute(user_pitch + correction, pitch, dt)
        
        # 5. Fisica (semplificata)
        accel = control_output + wind
        velocity += accel * dt
        pitch += velocity * dt
        
        # Log
        history['pitch'].append(pitch)
        history['correction'].append(correction)
        history['mode'].append(1 if pid.mode == 'DYNAMIC' else 0)
        history['wind'].append(wind)

    # Plotting
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
    ax1.plot(time, history['pitch'], label='Pitch Reale', color='blue', linewidth=2)
    ax1.axhline(y=user_pitch, color='red', linestyle='--', label='Setpoint')
    ax1.fill_between(time, 0, history['wind'], color='gray', alpha=0.2, label='Vento (Disturbo)')
    ax1.set_title("Simulazione Avanzata: Gain Scheduling + Disturbance Observer")
    ax1.set_ylabel("Gradi")
    ax1.legend()
    ax1.grid(True)

    ax2.plot(time, history['correction'], label='Correzione DOB', color='green')
    ax2.step(time, [m * 5 for m in history['mode']], where='post', label='Mode (0:Static, 5:Dyn)', color='orange', linestyle='--')
    ax2.set_title("Analisi Interna: Offset Observer e Stato Controller")
    ax2.set_xlabel("Tempo (s)")
    ax2.set_ylabel("Valore Correttivo")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.savefig('simulation/advanced_test_result.png')
    print("Test completato con logica reale. Grafico: simulation/advanced_test_result.png")

if __name__ == "__main__":
    run_advanced_sim()
