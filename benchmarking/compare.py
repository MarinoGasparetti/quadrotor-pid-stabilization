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
    """Simulazione del sistema con Disturbance Observer migliorata"""
    def __init__(self, kp, ki, kd, dob_gain=2.5, dt=0.01):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dob_gain = dob_gain
        self.dt = dt
        self.integral = 0
        self.prev_error = 0
        self.estimated_dist = 0

    def update(self, setpoint, measurement, actual_control_effort):
        error = setpoint - measurement
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        
        # Logica DOB migliorata: il controllo nominale include la componente derivativa
        nominal_control = self.kp * error + self.kd * derivative
        
        # Stima del disturbo più reattiva
        dist_obs = (nominal_control - actual_control_effort) * self.dob_gain
        self.estimated_dist += (dist_obs - self.estimated_dist) * 0.2
        
        self.prev_error = error
        # Sottraiamo il disturbo stimato per compensare
        return (self.kp * error + self.ki * self.integral + self.kd * derivative) - self.estimated_dist

def run_benchmark():
    dt = 0.01
    time = np.arange(0, 15, dt) # Aumentato tempo per vedere assestamento
    setpoint = 1.0
    
    # Parametri
    std_pid = StandardPID(2.0, 0.8, 0.15)
    adv_dob = AdvancedDOBController(2.0, 0.8, 0.15, dob_gain=3.0)
    
    results = {'std': [], 'adv': []}
    pos_std, pos_adv = 0, 0
    last_out_adv = 0 # Inizializzazione corretta
    
    for i, t in enumerate(time):
        # Disturbo a scalino dopo 5 secondi
        dist = 0.6 if t > 5 else 0.0
        
        # PID Standard
        out_std = std_pid.update(setpoint, pos_std)
        pos_std += (out_std - dist) * dt
        results['std'].append(pos_std)
        
        # Advanced DOB con feedback dell'ultimo output di controllo
        out_adv = adv_dob.update(setpoint, pos_adv, last_out_adv)
        pos_adv += (out_adv - dist) * dt
        results['adv'].append(pos_adv)
        last_out_adv = out_adv

    iae_std = np.sum(np.abs(setpoint - np.array(results['std'])))
    iae_adv = np.sum(np.abs(setpoint - np.array(results['adv'])))
    
    print(f"--- RISULTATI BENCHMARK ---")
    print(f"IAE (Integral Absolute Error) - PID Standard: {iae_std:.4f}")
    print(f"IAE (Integral Absolute Error) - Advanced DOB: {iae_adv:.4f}")
    
    improvement = ((iae_std - iae_adv) / iae_std) * 100
    print(f"Miglioramento: {improvement:.2f}%")
    
    if improvement < 15:
        print(f"BENCHMARK NON RAGGIUNTO: Miglioramento {improvement:.2f}% < 15%")
        exit(1)
    else:
        print(f"TEST SUPERATO: Miglioramento significativo ({improvement:.2f}%)")
        exit(0)

if __name__ == "__main__":
    run_benchmark()
