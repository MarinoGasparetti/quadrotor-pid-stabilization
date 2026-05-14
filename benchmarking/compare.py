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
    """Mediazione condizionata: scatta solo se il sistema viene da lontano (overshoot genuino).
    NON scatta se era già vicino al setpoint (potrebbe essere una raffica di vento).
    Usa una finestra storica di 1s per distinguere i due casi."""
    def __init__(self, lookback=100, approach_threshold=6.0):
        self.last_sign = 0
        self.error_buf = []
        self.lookback = lookback            # 1s di storia a dt=0.01
        self.approach_threshold = approach_threshold  # soglia in gradi

    def mediate(self, error, est_dist):
        current_sign = np.sign(error)
        self.error_buf.append(abs(error))
        if len(self.error_buf) > self.lookback:
            self.error_buf.pop(0)

        result = est_dist
        if self.last_sign != 0 and current_sign != self.last_sign:
            avg_recent = np.mean(self.error_buf) if self.error_buf else 0
            # Sistema arrivava da lontano (avg > soglia) → overshoot → media
            # Sistema era già vicino al setpoint (avg < soglia) → possibile gust → non mediare
            if avg_recent > self.approach_threshold:
                result = est_dist * 0.1

        self.last_sign = current_sign
        return result

class DOBControllerStabilized:
    def __init__(self, kp, ki, kd, dob_gain=0.25, dt=0.01):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dob_gain = dob_gain
        self.dt = dt
        self.integral = 0
        self.prev_error = None
        self.est_dist = 0
        self.eval_layer = ActiveEvaluationLayer()

    def update(self, setpoint, measurement, last_effort):
        error = setpoint - measurement
        if self.prev_error is None: self.prev_error = error

        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt

        nominal = (self.kp * error + self.kd * derivative)
        raw_dist = (nominal - last_effort) * self.dob_gain
        self.est_dist += (raw_dist - self.est_dist) * 0.7
        self.est_dist = np.clip(self.est_dist, -12.0, 12.0)

        # layer di mediazione: riduce la correzione al cambio segno errore
        # ma NON sovrascrive lo stato interno (l'observer mantiene memoria)
        mediated_dist = self.eval_layer.mediate(error, self.est_dist)

        self.prev_error = error
        return (self.kp * error + self.ki * self.integral + self.kd * derivative) - mediated_dist

def run_benchmark():
    dt = 0.01
    time = np.arange(0, 20, dt)
    setpoint = 20.0

    std_pid = StandardPID(1.5, 0.5, 2.5)
    dob_sys = DOBControllerStabilized(1.5, 0.5, 2.5, dob_gain=0.6)
    
    res_std, res_dob = [], []
    pos_std, pos_dob = 0.0, 0.0
    vel_std, vel_dob = 0.0, 0.0
    last_eff_dob = 0
    inertia = 0.8

    print(f"{'Time':<8} | {'Dist':<6} | {'PID Pos':<10} | {'DOB Pos':<10} | {'DOB Est Dist':<12}")
    print("-" * 55)

    for i, t in enumerate(time):
        dist = 12.0 if 3.0 <= t <= 5.0 else 0.0
        
        out_std = std_pid.update(setpoint, pos_std)
        accel_std = (out_std - dist) / inertia
        vel_std += accel_std * dt
        pos_std += vel_std * dt
        res_std.append(pos_std)
        
        out_dob = dob_sys.update(setpoint, pos_dob, last_eff_dob)
        accel_dob = (out_dob - dist) / inertia
        vel_dob += accel_dob * dt
        pos_dob += vel_dob * dt
        res_dob.append(pos_dob)
        last_eff_dob = out_dob

        if i % 100 == 0:
            print(f"{t:<8.2f} | {dist:<6.1f} | {pos_std:<10.2f} | {pos_dob:<10.2f} | {dob_sys.est_dist:<12.2f}")

    iae_std = np.sum(np.abs(np.array(res_std) - setpoint)) * dt
    iae_dob = np.sum(np.abs(np.array(res_dob) - setpoint)) * dt
    improvement = (iae_std - iae_dob) / iae_std * 100
    print(f"\nIAE PID Standard:  {iae_std:.2f}")
    print(f"IAE DOB Mediated:  {iae_dob:.2f}")
    print(f"Miglioramento IAE: {improvement:+.2f}%")
    print(f"CI gate (>15%):    {'PASSED' if improvement > 15 else 'FAILED'}")

    plt.figure(figsize=(12, 7))
    plt.plot(time, res_std, label=f'PID Standard (IAE={iae_std:.1f})', color='red', alpha=0.5)
    plt.plot(time, res_dob, label=f'DOB Mediated Reset (IAE={iae_dob:.1f})', color='blue', linewidth=2)
    plt.axhline(y=setpoint, color='black', linestyle='--', alpha=0.3)
    wind_profile = [12.0 if 3.0 <= t <= 5.0 else 0.0 for t in time]
    plt.ylim(-5, 35)
    plt.fill_between(time, 0, wind_profile, color='gray', alpha=0.15, label='Raffica Vento (N)')
    plt.title("Benchmark: DOB con Active Reset Mediato")
    plt.xlabel("Tempo (s)")
    plt.ylabel("Gradi Pitch")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.savefig('benchmarking/benchmark_rebound_fixed.png')

if __name__ == "__main__":
    run_benchmark()
