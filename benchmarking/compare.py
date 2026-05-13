import numpy as np
import matplotlib.pyplot as plt

class StandardPID:
    def __init__(self, kp, ki, kd, dt=0.01):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.integral = 0
        self.prev_error = 0

    def update(self, setpoint, measurement):
        error = setpoint - measurement
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class AdvancedDOBController:
    """Simulazione semplificata del sistema nel core/ con Disturbance Observer"""
    def __init__(self, kp, ki, kd, dob_gain=0.5, dt=0.01):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dob_gain = dob_gain
        self.dt = dt
        self.integral = 0
        self.prev_error = 0
        self.estimated_dist = 0

    def update(self, setpoint, measurement, actual_output):
        error = setpoint - measurement
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        
        # Logica DOB: stima il disturbo confrontando l'output atteso con quello reale
        # Nel sistema reale è implementato in disturbance_observer.hpp
        nominal_model_output = self.kp * error # Semplificato
        dist_obs = (nominal_model_output - actual_output) * self.dob_gain
        self.estimated_dist += (dist_obs - self.estimated_dist) * 0.1 # Filtro passa-basso
        
        self.prev_error = error
        return (self.kp * error + self.ki * self.integral + self.kd * derivative) - self.estimated_dist

def run_benchmark():
    dt = 0.01
    time = np.arange(0, 10, dt)
    setpoint = 1.0
    
    # Parametri
    std_pid = StandardPID(2.0, 0.5, 0.1)
    adv_dob = AdvancedDOBController(2.0, 0.5, 0.1, dob_gain=1.2)
    
    results = {'std': [], 'adv': []}
    pos_std, pos_adv = 0, 0
    
    for i, t in enumerate(time):
        # A 5 secondi inseriamo un disturbo costante (es. vento forte)
        dist = 0.5 if t > 5 else 0.0
        
        # PID Standard
        out_std = std_pid.update(setpoint, pos_std)
        pos_std += (out_std - dist) * dt
        results['std'].append(pos_std)
        
        # Advanced DOB
        out_adv = adv_dob.update(setpoint, pos_adv, out_adv if i>0 else 0)
        pos_adv += (out_adv - dist) * dt
        results['adv'].append(pos_adv)

    iae_std = np.sum(np.abs(setpoint - np.array(results['std'])))
    iae_adv = np.sum(np.abs(setpoint - np.array(results['adv'])))
    
    print(f"--- RISULTATI BENCHMARK ---")
    print(f"IAE (Integral Absolute Error) - PID Standard: {iae_std:.4f}")
    print(f"IAE (Integral Absolute Error) - Advanced DOB: {iae_adv:.4f}")
    
    improvement = ((iae_std - iae_adv) / iae_std) * 100
    print(f"Miglioramento: {improvement:.2f}%")
    
    if improvement < 15:
        print("TEST FALLITO: Il beneficio del DOB è troppo basso.")
        exit(1)
    else:
        print("TEST SUPERATO: Il sistema con DOB è significativamente più resiliente.")
        exit(0)

if __name__ == "__main__":
    run_benchmark()
