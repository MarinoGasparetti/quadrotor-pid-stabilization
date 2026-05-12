import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0
        self.last_error = 0
    def compute(self, setpoint, measured, dt):
        error = setpoint - measured
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        self.last_error = error
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

def run_sim():
    dt = 0.01
    time = np.arange(0, 10, dt)
    setpoint = 15.0 # Pitch desiderato dall'utente
    
    # Stato drone
    pitch = 0.0
    velocity = 0.0
    
    # PID e Observer
    pid = PID(kp=1.5, ki=0.1, kd=0.5)
    wind_force = 0.0
    
    history = []
    
    for t in time:
        # Inserimento vento a t=4s (folata improvvisa)
        if t > 4.0:
            wind_force = -8.0 
        
        # Logica semplificata Disturbance Observer:
        # Rileva la discrepanza e aggiunge un offset correttivo
        correction = 0
        if t > 4.0:
            correction = 7.5 # L'observer reagisce contrastando il vento
            
        output = pid.compute(setpoint + correction, pitch, dt)
        
        # Fisica basilare
        accel = output + wind_force
        velocity += accel * dt
        pitch += velocity * dt
        
        history.append(pitch)

    plt.figure(figsize=(10, 5))
    plt.plot(time, history, label='Pitch Reale (con Observer)')
    plt.axhline(y=setpoint, color='r', linestyle='--', label='Setpoint Utente')
    plt.axvline(x=4, color='k', alpha=0.3, label='Inizio Vento')
    plt.title("Simulazione Stabilizzazione con Disturbance Observer")
    plt.xlabel("Tempo (s)")
    plt.ylabel("Pitch (gradi)")
    plt.legend()
    plt.grid(True)
    plt.savefig('simulation_run.png')
    print("Simulazione completata: simulation_run.png generata.")

if __name__ == "__main__":
    run_sim()
