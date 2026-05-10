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
        
        # Matrici originali intatte
        out_a = (self.kp_a * error) + (self.kd_a * derivative)
        out_b = (self.kp_b * error) + (self.kd_b * derivative)

        # Calcolo dominanza
        dominance = np.tanh(abs(error) / self.threshold)
        
        # DEFINIZIONE LINEA DI SMUSSO (Transition Zone)
        # Creiamo un corridoio di transizione tra 0.45 e 0.55
        # Sotto 0.45 -> Beta puro
        # Sopra 0.55 -> Alpha puro
        # In mezzo -> Interpolazione lineare
        
        lower_bound = 0.45
        upper_bound = 0.55
        
        if dominance > upper_bound:
            output = out_a
            mode_val = 1.0  # Alpha
        elif dominance < lower_bound:
            output = out_b
            mode_val = 0.0  # Beta
        else:
            # Calcolo il peso locale dentro la zona di smusso (da 0 a 1)
            t = (dominance - lower_bound) / (upper_bound - lower_bound)
            output = (t * out_a) + ((1 - t) * out_b)
            mode_val = t  # Valore intermedio per il grafico

        self.prev_error = error
        return output, mode_val

def simulate():
    dt = 0.01
    t_end = 7.0
    time = np.arange(0, t_end, dt)
    setpoint = 10.0
    current_height = 0.0
    velocity = 0.0
    gravity = 9.81
    mass = 1.0

    dual_pd = DualOpposingPD(kp_a=18.0, kd_a=5.0, kp_b=10.0, kd_b=25.0, threshold=3.5)

    history = []
    modes = []

    for t in time:
        thrust, mode_val = dual_pd.compute(setpoint, current_height, dt)
        acceleration = (thrust / mass) - gravity
        velocity += acceleration * dt
        current_height += velocity * dt
        if current_height < 0:
            current_height = 0
            velocity = 0
        history.append(current_height)
        modes.append(mode_val)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    ax1.plot(time, history, label='Altezza Drone (Smoothed Transition)', color='b', lw=2)
    ax1.axhline(y=setpoint, color='r', linestyle='--', label='Setpoint')
    ax1.set_ylabel("Altezza (m)")
    ax1.set_title("Simulazione con Zona di Smusso (0.45 - 0.55)")
    ax1.legend()
    ax1.grid(True)
    
    # Ora il grafico dei modi non sarà più solo 0 o 1, ma vedremo la rampa
    ax2.plot(time, modes, label='Modo (1=Alpha, 0=Beta, middle=Smusso)', color='g')
    ax2.set_ylabel("Modo / Mix")
    ax2.set_xlabel("Tempo (s)")
    ax2.grid(True)
    ax2.legend()

    plt.tight_layout()
    plt.savefig('simulation_result.png')
    print("Simulazione completata. Applicata zona di smusso locale per eliminare il chatter.")

if __name__ == "__main__":
    simulate()
