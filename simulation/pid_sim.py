import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

class QuadrotorCoupledSim:
    """Simula il drone con accoppiamento tra gli assi (Rimbalzo in Aria)"""
    def __init__(self, kp, kd, coupling_factor=0.3):
        self.kp = kp
        self.kd = kd
        self.coupling = coupling_factor # Quanto il Pitch influisce sul Roll
        self.prev_pitch_err = 0
        self.prev_roll_err = 0
        self.initialized = False

    def compute(self, target, current_p, current_r, dt):
        # PID Pitch
        p_err = target[0] - current_p
        
        if not self.initialized:
            self.prev_pitch_err = p_err
            self.prev_roll_err = target[1] - current_r
            self.initialized = True
            
        p_der = (p_err - self.prev_pitch_err) / dt
        p_out = (self.kp * p_err) + (self.kd * p_der)
        
        # PID Roll (Cerca di stare a 0)
        r_err = target[1] - current_r
        r_der = (r_err - self.prev_roll_err) / dt
        r_out = (self.kp * r_err) + (self.kd * r_der)

        # Accoppiamento: Una correzione forte di Pitch genera disturbo su Roll
        # Simula asimmetria motori durante manovre brusche
        roll_disturbance = p_out * self.coupling
        
        self.prev_pitch_err = p_err
        self.prev_roll_err = r_err
        
        return p_out, r_out + roll_disturbance

def run_simulation():
    dt = 0.01
    time = np.arange(0, 8, dt)
    target = [15.0, 0.0] # 15° Pitch, 0° Roll
    
    # Stato: [pitch, roll, vel_p, vel_r]
    state = np.array([0.0, 0.0, 0.0, 0.0])
    # Gain volutamente aggressivi per mostrare l'oscillazione
    sim = QuadrotorCoupledSim(kp=2.5, kd=0.6, coupling_factor=0.5)
    
    history = []
    
    for t in time:
        p_out, r_out = sim.compute(target, state[0], state[1], dt)
        
        # Dinamica con Inerzia e smorzamento aerodinamico
        # Accel = (Coppia Motori - Attrito Aria) / Inerzia
        accel_p = (p_out - (state[2] * 0.15)) 
        accel_r = (r_out - (state[3] * 0.15))
        
        state[2] += accel_p * dt
        state[3] += accel_r * dt
        state[0] += state[2] * dt
        state[1] += state[3] * dt
        
        history.append(state.copy())

    history = np.array(history)
    
    plt.figure(figsize=(12, 7))
    plt.subplot(2, 1, 1)
    plt.plot(time, history[:, 0], label='Pitch Ang (deg)', color='blue', linewidth=2)
    plt.axhline(y=target[0], color='red', linestyle='--', label='Target Pitch')
    plt.title("Simulazione Rimbalzo: Stabilizzazione Pitch con Accoppiamento")
    plt.ylabel("Gradi")
    plt.legend()
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(time, history[:, 1], label='Roll Ang (deg)', color='orange', linewidth=2)
    plt.axhline(y=target[1], color='black', linestyle='--', label='Target Roll')
    plt.title("Effetto Parassita sul Roll (Rimbalzo in Aria)")
    plt.xlabel("Tempo (s)")
    plt.ylabel("Gradi")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig('simulation/coupled_rebound_sim.png')
    print("Simulazione completata. Grafico: simulation/coupled_rebound_sim.png")

if __name__ == "__main__":
    run_simulation()
