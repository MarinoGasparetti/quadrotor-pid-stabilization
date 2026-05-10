# Quadrotor PID Stabilization

Simulazione e implementazione di controllori PID per la stabilizzazione di un quadricottero.

## Struttura

- `core/` — Header C++ con i controllori PID
  - `pid_controller.hpp` — PID classico con clamping
  - `experimental_pd.hpp` — Dual Opposing PD sperimentale
- `simulation/` — Simulazione Python
  - `pid_sim.py` — Simulazione Dual Opposing PD con grafici

## Esecuzione simulazione

```bash
pip install numpy matplotlib
python simulation/pid_sim.py
```
