import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

class StandardPID:
    def __init__(self, kp, ki, kd, dt=0.01):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.integral = 0
        self.prev_error = None # Fix: Inizializzazione lazy

    def update(self, setpoint, measurement):
        error = setpoint - measurement
        if self.prev_error is None:
            self.prev_error = error
            
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class DOBControllerStabilized:
    def __init__(self, kp, ki, kd, dob_gain=0.2, dt=0.01):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dob_gain = dob_gain 
        self.dt = dt
        self.integral = 0
        self.prev_error = None # Fix 1: Inizializzazione lazy per evitare spike a t=0
        self.est_dist = 0

    def update(self, setpoint, measurement, last_effort):
        error = setpoint - measurement
        if self.prev_error is None:
            self.prev_error = error
            
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        
        # Modello nominale
        nominal = (self.kp * error + self.kd * derivative)
        
        # Stima del disturbo
        raw_dist = (nominal - last_effort) * self.dob_gain
        
        # Fix 2: Aumento reattività del filtro (0.05 -> 0.12) 
        # per ridurre il ritardo nella stima quando il vento cessa.
        self.est_dist += (raw_dist - self.est_dist) * 0.12
        
        # Clamping bilanciato
        self.est_dist = np.clip(self.est_dist, -12.0, 12.0)
        
        self.prev_error = error
        return (self.kp * error + self.ki * self.integral + self.kd * derivative) - self.est_dist

def run_benchmark():
    dt = 0.01
    time = np.arange(0, 10, dt)
    setpoint = 20.0 
    
    # Tuning PID più conservativo per evidenziare il lavoro del DOB
    std_pid = StandardPID(1.2, 0.4, 0.3)
    dob_sys = DOBControllerStabilized(1.2, 0.4, 0.3, dob_gain=0.25)
    
    res_std, res_dob = [], []
    pos_std, pos_dob = 0.0, 0.0
    vel_std, vel_dob = 0.0, 0.0
    last_eff_dob = 0
    inertia = 0.8

    print(f"{'Time':<8} | {'Dist':<6} | {'PID Pos':<10} | {'DOB Pos':<10} | {'DOB Est Dist':<12}")
    print("-" * 55)

    for i, t in enumerate(time):
        # Disturbo impulsivo
        dist = 12.0 if 3.0 <= t <= 5.0 else 0.0
        
        # PID
        out_std = std_pid.update(setpoint, pos_std)
        accel_std = (out_std - dist) / inertia
        vel_std += accel_std * dt
        pos_std += vel_std * dt
        res_std.append(pos_std)
        
        # DOB
        out_dob = dob_sys.update(setpoint, pos_dob, last_eff_dob)
        accel_dob = (out_dob - dist) / inertia
        vel_dob += accel_dob * dt
        pos_dob += vel_dob * dt
        res_dob.append(pos_dob)
        last_eff_dob = out_dob

        if i % 50 == 0:
            print(f"{t:<8.2f} | {dist:<6.1f} | {pos_std:<10.2f} | {pos_dob:<10.2f} | {dob_sys.est_dist:<12.2f}")

    plt.figure(figsize=(12, 7))
    plt.plot(time, res_std, label='PID Standard', color='red', alpha=0.6)
    plt.plot(time, res_dob, label='DOB Proattivo (Fixed)', color='blue', linewidth=2)
    plt.axhline(y=setpoint, color='black', linestyle='--', alpha=0.3)
    plt.fill_between(time, 0, 12, where=((time >= 3) & (time <= 5)), color='gray', alpha=0.1, label='Vento')
    
    plt.title("Benchmark FIX 2: Rimozione Spike Iniziale e Ottimizzazione Transitorio")
    plt.xlabel("Tempo (s)")
    plt.ylabel("Gradi Pitch")
    plt.legend()
    plt.grid(True, alpha=0.2)
    
    plt.savefig('benchmarking/benchmark_rebound_fixed.png')
    print("\nAnalisi completata. Workflow pronto per verifica.")

if __name__ == "__main__":
    run_benchmark()
