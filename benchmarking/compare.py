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

class DOBControllerWithRebound:
    """DOB con compensazione dei rimbalzi (filtro predittivo)"""
    def __init__(self, kp, ki, kd, dob_gain=0.6, dt=0.01):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dob_gain = dob_gain
        self.dt = dt
        self.integral = 0
        self.prev_error = 0
        self.est_dist = 0

    def update(self, setpoint, measurement, last_effort):
        error = setpoint - measurement
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        
        # Stima del disturbo basata sulla discrepanza tra comando e moto reale
        # Includiamo un termine di smorzamento per i rimbalzi
        nominal = (self.kp * error + self.kd * derivative)
        raw_dist = (nominal - last_effort) * self.dob_gain
        
        # Low pass filter sulla stima per evitare risonanze (rimbalzi)
        self.est_dist += (raw_dist - self.est_dist) * 0.15
        
        self.prev_error = error
        return (self.kp * error + self.ki * self.integral + self.kd * derivative) - self.est_dist

def run_benchmark():
    dt = 0.01
    time = np.arange(0, 10, dt)
    setpoint = 20.0 # Target 20 gradi
    
    std_pid = StandardPID(1.5, 0.5, 0.3)
    dob_sys = DOBControllerWithRebound(1.5, 0.5, 0.3, dob_gain=0.7)
    
    res_std, res_dob = [], []
    pos_std, pos_dob = 0.0, 0.0
    vel_std, vel_dob = 0.0, 0.0
    last_eff_dob = 0
    
    # Inerzia del drone
    inertia = 0.8

    for t in time:
        # Disturbo: Raffica di vento istantanea a t=3s
        dist = 8.0 if 3.0 <= t <= 4.0 else 0.0
        
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

    # Grafico di confronto
    plt.figure(figsize=(12, 7))
    plt.plot(time, res_std, label='Standard PID (Overshoot visibile)', color='red', alpha=0.6)
    plt.plot(time, res_dob, label='DOB Enhanced (Rimbalzo attenuato)', color='blue', linewidth=2)
    plt.axhline(y=setpoint, color='black', linestyle='--', label='Target')
    plt.fill_between(time, 0, 25, where=((time >= 3) & (time <= 4)), color='gray', alpha=0.2, label='Raffica Vento')
    
    plt.title("Benchmark: Risposta ai rimbalzi aerodinamici (Pitch Stabilization)")
    plt.xlabel("Tempo (s)")
    plt.ylabel("Gradi Pitch")
    plt.legend()
    plt.grid(True)
    
    plt.savefig('benchmarking/benchmark_rebound.png')
    print("Benchmark completato. Grafico: benchmarking/benchmark_rebound.png")

if __name__ == "__main__":
    run_benchmark()
