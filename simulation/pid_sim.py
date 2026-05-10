import numpy as np
import matplotlib
# Il backend deve essere impostato PRIMA di importare pyplot
matplotlib.use('Agg')
import matplotlib.pyplot as plt

class DualOpposingPD:
    def __init__(self, kp_a, kd_a, kp_b, kd_b, threshold=1.0):
        self.kp_a = kp_a
        self.kd_a = kd_a
        self.kp_b = kp_b
        self.kd_b = kd_b
        self.threshold = threshold
        self.prev_error = 0

    def compute(self, setpoint, measurement, dt):
        error = setpoint - measurement
        derivative = (error - self.prev_error) / dt
        
        # Calcolo i due output teorici (le matrici restano intatte)
        out_a = (self.kp_a * error) + (self.kd_a * derivative)
        out_b = (self.kp_b * error) + (self.kd_b * derivative)

        # Calcolo il fattore di transizione (0.0 = Pure Beta, 1.0 = Pure Alpha)
        # L'uso di tanh garantisce una transizione morbida ma preserva gli estremi
        dominance = np.tanh(abs(error) / self.threshold)
        
        # Interpolazione Lineare (LERP) per stabilizzare la transizione
        output = (dominance * out_a) + ((1 - dominance) * out_b)

        self.prev_error = error
        return output, dominance

def simulate():
    dt = 0.01
    t_end = 7.0
    time = np.arange(0, t_end, dt)
    setpoint = 10.0
    current_height = 0.0
    velocity = 0.0
    gravity = 9.81
    mass = 1.0

    # Mantengo i parametri originali come richiesto
    dual_pd = DualOpposingPD(kp_a=18.0, kd_a=5.0, kp_b=10.0, kd_b=25.0, threshold=3.5)

    history = []
    dominance_history = []

    for t in time:
        thrust, dominance = dual_pd.compute(setpoint, current_height, dt)
        acceleration = (thrust / mass) - gravity
        velocity += acceleration * dt
        current_height += velocity * dt
        if current_height < 0:
            current_height = 0
            velocity = 0
        history.append(current_height)
        dominance_history.append(dominance)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    ax1.plot(time, history, label='Altezza Drone (Smoothed Transition)', color='b', lw=2)
    ax1.axhline(y=setpoint, color='r', linestyle='--', label='Setpoint')
    ax1.set_ylabel("Altezza (m)")
    ax1.set_title("Simulazione Dual Opposing PD con Interpolazione Lineare")
    ax1.legend()
    ax1.grid(True)
    
    ax2.plot(time, dominance_history, label='Mix Alpha/Beta (1.0=Alpha, 0.0=Beta)', color='purple')
    ax2.set_ylabel("Dominanza")
    ax2.set_xlabel("Tempo (s)")
    ax2.grid(True)
    ax2.legend()

    plt.tight_layout()
    plt.savefig('simulation_result.png')
    print("Ottimizzazione completata. Transizioni stabilizzate tramite interpolazione.")

if __name__ == "__main__":
    simulate()
