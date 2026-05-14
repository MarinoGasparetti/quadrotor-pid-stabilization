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
    def __init__(self, kp, ki, kd, dob_gain=0.15, dt=0.01):
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
        nominal = (self.kp * error + self.kd * derivative)
        raw_dist = (nominal - last_effort) * self.dob_gain
        self.est_dist += (raw_dist - self.est_dist) * 0.05
        self.est_dist = np.clip(self.est_dist, -10.0, 10.0)
        self.prev_error = error
        return (self.kp * error + self.ki * self.integral + self.kd * derivative) - self.est_dist

def run_benchmark():
    dt = 0.01
    time = np.arange(0, 10, dt)
    setpoint = 20.0 
    
    std_pid = StandardPID(1.5, 0.5, 0.4)
    dob_sys = DOBControllerStabilized(1.5, 0.5, 0.4, dob_gain=0.15)
    
    res_std, res_dob = [], []
    pos_std, pos_dob = 0.0, 0.0
    vel_std, vel_dob = 0.0, 0.0
    last_eff_dob = 0
    inertia = 0.8

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
        
        # DOB
        out_dob = dob_sys.update(setpoint, pos_dob, last_eff_dob)
        accel_dob = (out_dob - dist) / inertia
        vel_dob += accel_dob * dt
        pos_dob += vel_dob * dt
        res_dob.append(pos_dob)
        last_eff_dob = out_dob

        # Log ogni 0.5s
        if i % 50 == 0:
            print(f"{t:<8.2f} | {dist:<6.1f} | {pos_std:<10.2f} | {pos_dob:<10.2f} | {dob_sys.est_dist:<12.2f}")

    plt.figure(figsize=(12, 7))
    plt.plot(time, res_std, label='PID')
    plt.plot(time, res_dob, label='DOB (Fixed)')
    plt.axhline(y=setpoint, color='black', linestyle='--')
    plt.savefig('benchmarking/benchmark_rebound_fixed.png')
    print("\nBenchmark Fix completato con successo.")

if __name__ == "__main__":
    run_benchmark()
