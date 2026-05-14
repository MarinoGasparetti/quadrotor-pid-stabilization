import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

class WindDisturbanceSim:
    """Simula l'effetto della logica DisturbanceObserver su un drone"""
    def __init__(self, kp, kd, wind_sensitivity=0.5):
        self.kp = kp
        self.kd = kd
        self.alpha = wind_sensitivity 
        self.prev_error = 0
        self.initialized = False

    def compute(self, user_pitch, current_pitch, dt, external_wind):
        error = user_pitch - current_pitch
        
        # Inizializzazione per evitare spike a t=0
        if not self.initialized:
            self.prev_error = error
            self.initialized = True
            return 0.0, 0.0

        # Calcolo derivata errore marginale
        derivative_error = (error - self.prev_error) / dt
        
        # Correzione con clamping a +/- 20.0 (limite fisico motori)
        correction = np.clip(self.alpha * derivative_error, -20.0, 20.0)
        
        effective_setpoint = user_pitch + correction
        output = (self.kp * (effective_setpoint - current_pitch)) + (self.kd * derivative_error)
        
        self.prev_error = error
        return output, correction

def run_simulation():
    dt = 0.01
    time = np.arange(0, 10, dt)
    user_pitch = 20.0  
    current_pitch = 0.0
    velocity = 0.0
    
    sim = WindDisturbanceSim(kp=1.5, kd=0.4, wind_sensitivity=0.8)
    sim.prev_error = user_pitch - current_pitch  # evita spike derivativo al t=0
    
    pitches = []
    corrections = []
    wind_profile = []

    for i, t in enumerate(time):
        wind = 15.0 if 4.0 <= t <= 6.0 else 0.0
        disturbed_pitch = current_pitch - (wind * 0.05)
        
        accel, corr = sim.compute(user_pitch, disturbed_pitch, dt, wind)
        
        velocity += accel * dt
        current_pitch += velocity * dt
        
        pitches.append(current_pitch)
        corrections.append(corr)
        wind_profile.append(wind)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
    
    ax1.plot(time, pitches, label='Pitch Reale del Drone', color='blue')
    ax1.axhline(y=user_pitch, color='red', linestyle='--', label='Target Utente')
    ax1.fill_between(time, 4, 6, color='gray', alpha=0.2, label='Raffica di Vento')
    ax1.set_title("Risposta al Vento con Disturbance Observer (Senza Spike)")
    ax1.set_ylabel("Gradi Pitch")
    ax1.legend()
    ax1.grid(True)

    ax2.plot(time, corrections, label='Correzione Observer (Offset)', color='green')
    ax2.set_title("Sforzo dell'Observer (Clamped & Smooth)")
    ax2.set_ylabel("Offset Correttivo")
    ax2.set_xlabel("Tempo (s)")
    ax2.set_ylim(-25, 25) # Zoom sul range reale
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.savefig('simulation/wind_rejection_result.png')
    print("Simulazione completata. Risultati in simulation/wind_rejection_result.png")

if __name__ == "__main__":
    run_simulation()
