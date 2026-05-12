import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

class WindDisturbanceSim:
    """Simula l'effetto della logica DisturbanceObserver su un drone"""
    def __init__(self, kp, kd, wind_sensitivity=0.5):
        self.kp = kp
        self.kd = kd
        self.alpha = wind_sensitivity # Corrisponde al guadagno dell'Observer
        self.prev_error = 0
        self.integral_correction = 0

    def compute(self, user_pitch, current_pitch, dt, external_wind):
        # Errore base
        error = user_pitch - current_pitch
        
        # Logica Disturbance Observer:
        # Calcoliamo la derivata dell'errore marginale (la "mutazione")
        derivative_error = (error - self.prev_error) / dt
        
        # Se il vento (external_wind) spinge, la derivata dell'errore cambia bruscamente.
        # L'observer inietta una correzione proporzionale alla derivata per anticipare il vento.
        correction = self.alpha * derivative_error
        
        # Il setpoint reale passato ai motori include la correzione per il vento
        effective_setpoint = user_pitch + correction
        
        # Calcolo PID classico sul setpoint corretto
        output = (self.kp * (effective_setpoint - current_pitch)) + (self.kd * derivative_error)
        
        self.prev_error = error
        return output, correction

def run_simulation():
    dt = 0.01
    time = np.arange(0, 10, dt)
    user_pitch = 20.0  # L'utente vuole andare avanti a 20 gradi
    current_pitch = 0.0
    velocity = 0.0
    
    sim = WindDisturbanceSim(kp=1.5, kd=0.4, wind_sensitivity=0.8)
    
    pitches = []
    corrections = []
    wind_profile = []

    for i, t in enumerate(time):
        # Generiamo una raffica di vento improvvisa tra i 4 e i 6 secondi
        wind = 15.0 if 4.0 <= t <= 6.0 else 0.0
        
        # Il drone subisce il vento (che tende a raddrizzarlo/spostarlo)
        disturbed_pitch = current_pitch - (wind * 0.05)
        
        # Il controller reagisce
        accel, corr = sim.compute(user_pitch, disturbed_pitch, dt, wind)
        
        velocity += accel * dt
        current_pitch += velocity * dt
        
        pitches.append(current_pitch)
        corrections.append(corr)
        wind_profile.append(wind)

    # Grafici
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
    
    ax1.plot(time, pitches, label='Pitch Reale del Drone', color='blue')
    ax1.axhline(y=user_pitch, color='red', linestyle='--', label='Target Utente')
    ax1.fill_between(time, 0, wind_profile, color='gray', alpha=0.2, label='Raffica di Vento')
    ax1.set_title("Risposta al Vento con Disturbance Observer")
    ax1.set_ylabel("Gradi Pitch")
    ax1.legend()
    ax1.grid(True)

    ax2.plot(time, corrections, label='Correzione Observer (Offset)', color='green')
    ax2.set_title("Sforzo dell'Observer per mantenere la traiettoria")
    ax2.set_ylabel("Offset Correttivo")
    ax2.set_xlabel("Tempo (s)")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.savefig('simulation/wind_rejection_result.png')
    print("Simulazione completata. Risultati in simulation/wind_rejection_result.png")

if __name__ == "__main__":
    run_simulation()
