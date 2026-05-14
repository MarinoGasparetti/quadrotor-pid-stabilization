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

class ActiveEvaluationLayer:
    """
    Strato di valutazione attivo tra PID e Disturbance Observer.
    Valuta se la compensazione è coerente con la dinamica attuale.
    """
    def __init__(self, trust_threshold=0.8):
        self.trust_threshold = trust_threshold

    def evaluate(self, pid_output, est_dist, error_trend):
        # Se l'errore sta diminuendo drasticamente (il drone sta tornando), 
        # riduciamo il peso della compensazione esterna per evitare l'overshoot (frenata attiva).
        # error_trend < 0 significa che stiamo tornando verso il setpoint.
        
        compensation_weight = 1.0
        if error_trend < -0.5: # Ritorno veloce
            compensation_weight = 0.3 # 'Freniamo' l'observer
        elif abs(pid_output) > 20: # PID saturo, abbiamo bisogno di tutto l'aiuto possibile
            compensation_weight = 1.2 # 'Apriamo' i rubinetti del DOB
            
        return est_dist * compensation_weight

class HybridDOBController:
    def __init__(self, kp, ki, kd, dob_gain=0.25, dt=0.01):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dob_gain = dob_gain 
        self.dt = dt
        self.integral = 0
        self.prev_error = None
        self.est_dist = 0
        self.evaluator = ActiveEvaluationLayer()

    def update(self, setpoint, measurement, last_effort):
        error = setpoint - measurement
        if self.prev_error is None: self.prev_error = error
        
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        error_trend = derivative # Semplificato: la derivata dell'errore è il nostro trend
        
        # 1. Calcolo PID Classico
        pid_out = (self.kp * error + self.ki * self.integral + self.kd * derivative)
        
        # 2. Disturbance Observer (Stima grezza)
        nominal = pid_out
        raw_dist = (nominal - last_effort) * self.dob_gain
        self.est_dist += (raw_dist - self.est_dist) * 0.15 # Filtro reattivo
        self.est_dist = np.clip(self.est_dist, -15.0, 15.0)
        
        # 3. Layer di Valutazione Attivo (Decision Layer)
        # Decide quanto della stima del disturbo applicare effettivamente
        compensated_dist = self.evaluator.evaluate(pid_out, self.est_dist, error_trend)
        
        self.prev_error = error
        
        # Output finale: PID - Compensazione Valutata
        return pid_out - compensated_dist

def run_benchmark():
    dt = 0.01
    time = np.arange(0, 10, dt)
    setpoint = 20.0 
    
    std_pid = StandardPID(1.5, 0.5, 0.4)
    hybrid_sys = HybridDOBController(1.5, 0.5, 0.4, dob_gain=0.25)
    
    res_std, res_dob = [], []
    pos_std, pos_dob = 0.0, 0.0
    vel_std, vel_dob = 0.0, 0.0
    last_eff_hybrid = 0
    inertia = 0.8

    print(f"{'Time':<8} | {'Dist':<6} | {'PID Pos':<10} | {'Hybrid Pos':<10} | {'Comp Weight':<12}")
    print("-" * 65)

    for i, t in enumerate(time):
        dist = 12.0 if 3.0 <= t <= 5.0 else 0.0
        
        # PID Standard
        out_std = std_pid.update(setpoint, pos_std)
        accel_std = (out_std - dist) / inertia
        vel_std += accel_std * dt
        pos_std += vel_std * dt
        res_std.append(pos_std)
        
        # Hybrid DOB with Active Layer
        out_hybrid = hybrid_sys.update(setpoint, pos_dob, last_eff_hybrid)
        accel_dob = (out_hybrid - dist) / inertia
        vel_dob += accel_dob * dt
        pos_dob += vel_dob * dt
        res_dob.append(pos_dob)
        last_eff_hybrid = out_hybrid

        if i % 50 == 0:
            # Calcolo un peso approssimativo per il print log
            error_trend = ( (setpoint - pos_dob) - (hybrid_sys.prev_error if hybrid_sys.prev_error else 0) ) / dt
            weight = 0.3 if error_trend < -0.5 else (1.2 if abs(out_hybrid) > 20 else 1.0)
            print(f"{t:<8.2f} | {dist:<6.1f} | {pos_std:<10.2f} | {pos_dob:<10.2f} | {weight:<12.1f}")

    plt.figure(figsize=(12, 7))
    plt.plot(time, res_std, label='PID Standard', color='red', alpha=0.5)
    plt.plot(time, res_dob, label='Hybrid System (Active Layer)', color='green', linewidth=2)
    plt.axhline(y=setpoint, color='black', linestyle='--', alpha=0.3)
    plt.fill_between(time, 0, 12, where=((time >= 3) & (time <= 5)), color='gray', alpha=0.1, label='Vento')
    
    plt.title("Benchmark: Hybrid PID+DOB con Active Evaluation Layer")
    plt.xlabel("Tempo (s)")
    plt.ylabel("Gradi Pitch")
    plt.legend()
    plt.grid(True, alpha=0.2)
    
    plt.savefig('benchmarking/benchmark_hybrid_active.png')
    print("\nBenchmark Hybrid completato.")

if __name__ == "__main__":
    run_benchmark()
