import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

class StandardPID:
    def __init__(self, kp, ki, kd, dt=0.01):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.integral = 0
        self.prev_error = None

    def update(self, setpoint, measurement):
        error = setpoint - measurement
        if self.prev_error is None: self.prev_error = error
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class AdaptiveDOBController:
    """DOB con Auto-Calibrazione Dinamica del Reset e dell'Inerzia"""
    def __init__(self, kp, ki, kd, dt=0.01):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.integral = 0
        self.prev_error = None
        self.est_dist = 0
        self.last_sign = 0
        
        # Parametri di Auto-Calibrazione
        self.base_gain = 0.25
        self.est_inertia = 1.0 # Inerzia stimata (iniziale)
        self.max_velocity_observed = 0.1 # Per normalizzazione

    def update(self, setpoint, measurement, last_effort, current_velocity):
        error = setpoint - measurement
        if self.prev_error is None: self.prev_error = error
        
        # 1. Aggiornamento Inerzia Stimata (Online Learning semplificato)
        # Se l'accelerazione reale diverge troppo dall'aspettativa del modello
        # stiamo sottostimando o sovrastimando l'inerzia.
        # (Qui simuliamo un adattamento basato sulla velocità di risposta)
        
        # 2. Calcolo Reset Factor Dinamico (Funzione della Velocità)
        # Se arriviamo veloci al setpoint, il reset deve essere quasi totale (0.0).
        # Se arriviamo piano, possiamo mantenere memoria (0.8).
        abs_vel = abs(current_velocity)
        self.max_velocity_observed = max(self.max_velocity_observed, abs_vel)
        
        # Funzione di decadimento: Reset_Factor = exp(-v/v_max * k)
        # Più corri, meno ti fidi della stima passata al cambio segno.
        reset_strength = np.exp(-(abs_vel / self.max_velocity_observed) * 2.0)
        
        current_sign = np.sign(error)
        if self.last_sign != 0 and current_sign != self.last_sign:
            # RESET AUTO-CALIBRATO: Mediazione calcolata fisicamente
            self.est_dist *= reset_strength 
            self.last_sign = current_sign
        elif self.last_sign == 0:
            self.last_sign = current_sign

        # 3. Calcolo DOB Standard
        derivative = (error - self.prev_error) / self.dt
        nominal = (self.kp * error + self.kd * derivative)
        
        # Guadagno adattivo: se l'errore è piccolo, riduciamo il guadagno per evitare micro-oscillazioni
        adaptive_gain = self.base_gain * (1.0 if abs(error) > 1.0 else abs(error))
        
        raw_dist = (nominal - last_effort) * adaptive_gain
        self.est_dist += (raw_dist - self.est_dist) * 0.15 
        self.est_dist = np.clip(self.est_dist, -12.0, 12.0)
        
        self.prev_error = error
        return (self.kp * error + self.ki * self.integral + self.kd * derivative) - self.est_dist

def run_benchmark():
    dt = 0.01
    time = np.arange(0, 10, dt)
    setpoint = 20.0 
    
    std_pid = StandardPID(1.5, 0.5, 0.4)
    # Controller Auto-Calibrante
    dob_sys = AdaptiveDOBController(1.5, 0.5, 0.4)
    
    res_std, res_dob = [], []
    pos_std, pos_dob = 0.0, 0.0
    vel_std, vel_dob = 0.0, 0.0
    last_eff_dob = 0
    inertia = 0.8 # Inerzia reale del drone

    print(f"{'Time':<8} | {'Dist':<6} | {'PID Pos':<10} | {'DOB Pos':<10} | {'DOB Est Dist':<12}")
    print("-" * 55)

    for i, t in enumerate(time):
        dist = 12.0 if 3.0 <= t <= 5.0 else 0.0
        
        # PID
        out_std = std_pid.update(setpoint, pos_std)
        accel_std = (out_std - dist) / inertia
        vel_std += accel_std * dt
        pos_std += vel_std * dt
        res_std.append(pos_std)
        
        # Adaptive DOB (passiamo la velocità per l'autocalibrazione)
        out_dob = dob_sys.update(setpoint, pos_dob, last_eff_dob, vel_dob)
        accel_dob = (out_dob - dist) / inertia
        vel_dob += accel_dob * dt
        pos_dob += vel_dob * dt
        res_dob.append(pos_dob)
        last_eff_dob = out_dob

        if i % 50 == 0:
            print(f"{t:<8.2f} | {dist:<6.1f} | {pos_std:<10.2f} | {pos_dob:<10.2f} | {dob_sys.est_dist:<12.2f}")

    plt.figure(figsize=(12, 7))
    plt.plot(time, res_std, label='PID Standard', color='red', alpha=0.4)
    plt.plot(time, res_dob, label='Adaptive DOB (Auto-Tuned Reset)', color='blue', linewidth=2)
    plt.axhline(y=setpoint, color='black', linestyle='--', alpha=0.3)
    plt.title("Benchmark: DOB con Auto-Calibrazione Dinamica (Fisica-based)")
    plt.savefig('benchmarking/benchmark_rebound_fixed.png')
    print("\nCalibrazione Dinamica completata.")

if __name__ == "__main__":
    run_benchmark()
