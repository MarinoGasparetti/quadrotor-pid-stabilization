import numpy as np
import matplotlib
matplotlib.use('Agg')
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

class DOBControllerStabilized:
    """DOB con Clamping e Gain sintonizzato per evitare divergenza"""
    def __init__(self, kp, ki, kd, dob_gain=0.15, dt=0.01):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dob_gain = dob_gain # Ridotto per stabilità con inerzia
        self.dt = dt
        self.integral = 0
        self.prev_error = 0
        self.est_dist = 0

    def update(self, setpoint, measurement, last_effort):
        error = setpoint - measurement
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        
        # Modello nominale
        nominal = (self.kp * error + self.kd * derivative)
        
        # Stima del disturbo
        raw_dist = (nominal - last_effort) * self.dob_gain
        
        # 1. Filtro passa-basso (smorza i rimbalzi)
        self.est_dist += (raw_dist - self.est_dist) * 0.05
        
        # 2. Hard Clamping: evita l'esplosione della correzione
        self.est_dist = np.clip(self.est_dist, -10.0, 10.0)
        
        self.prev_error = error
        return (self.kp * error + self.ki * self.integral + self.kd * derivative) - self.est_dist

def run_benchmark():
    dt = 0.01
    time = np.arange(0, 20, dt)
    setpoint = 20.0 
    
    std_pid = StandardPID(1.2, 0.3, 1.8)
    dob_sys = DOBControllerStabilized(1.2, 0.3, 1.8, dob_gain=0.15)

    res_std, res_dob = [], []
    pos_std, pos_dob = 0.0, 0.0
    vel_std, vel_dob = 0.0, 0.0
    last_eff_dob = 0
    inertia = 0.8

    for t in time:
        # Disturbo reale (Vento)
        dist = 12.0 if 3.0 <= t <= 5.0 else 0.0
        
        # PID Standard
        out_std = std_pid.update(setpoint, pos_std)
        accel_std = (out_std - dist) / inertia
        vel_std += accel_std * dt
        pos_std += vel_std * dt
        res_std.append(pos_std)
        
        # DOB Controller
        out_dob = dob_sys.update(setpoint, pos_dob, last_eff_dob)
        accel_dob = (out_dob - dist) / inertia
        vel_dob += accel_dob * dt
        pos_dob += vel_dob * dt
        res_dob.append(pos_dob)
        last_eff_dob = out_dob

    # Grafico
    plt.figure(figsize=(12, 7))
    plt.plot(time, res_std, label='Standard PID (Drift/Overshoot)', color='red', alpha=0.7)
    plt.plot(time, res_dob, label='DOB Stabilizzato (Fixed)', color='blue', linewidth=2)
    plt.axhline(y=setpoint, color='black', linestyle='--', alpha=0.5)
    
    # 3. Fix cosmetico fill_between con profilo reale
    wind_profile = [12.0 if 3.0 <= t <= 5.0 else 0.0 for t in time]
    plt.fill_between(time, 0, wind_profile, color='gray', alpha=0.2, label='Raffica Vento (N)')
    
    plt.title("Benchmark FIX: Stabilizzazione con Inerzia e Clamping DOB")
    plt.xlabel("Tempo (s)")
    plt.ylabel("Gradi Pitch")
    plt.ylim(-5, 30)
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.savefig('benchmarking/benchmark_rebound_fixed.png')
    print("Benchmark Fix completato con successo.")

if __name__ == "__main__":
    run_benchmark()
