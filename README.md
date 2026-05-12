# Quadrotor PID Stabilization — Advanced Concurrency & Wind Rejection

Sistema di controllo avanzato per droni, ottimizzato per la stabilità in hovering e la precisione in volo dinamico attraverso l'uso di logiche concorrenti e compensazione dei disturbi ambientali.

## Caratteristiche Principali

- **Gain Scheduling (Static/Dynamic)**: Il `PIDController` cambia automaticamente i parametri tra hovering (precisione statica) e volo traslato (reattività dinamica).
- **Disturbance Observer**: Un modulo dedicato che analizza la derivata dell'errore marginale tra il comando utente e la risposta dell'IMU per rilevare e contrastare istantaneamente folate di vento.
- **DroneScheduler (Multi-threaded)**:
  - **Inner Loop (400Hz)**: Task ad alta priorità per la stabilizzazione dell'assetto.
  - **Outer Loop (50Hz)**: Task per la logica di navigazione e compensazione del vento.

## Struttura del Progetto

- `core/` — Logica di controllo in C++ (Header-only)
  - `pid_controller.hpp`: PID avanzato con supporto profili duali.
  - `disturbance_observer.hpp`: Calcolo della correzione predittiva basata sulla traiettoria utente.
  - `scheduler.hpp`: Orchestratore multi-thread per loop a frequenze differenziate.
- `simulation/` — Test e Validazione
  - `pid_sim.py`: Simulatore Python che ora include la modellazione di raffiche di vento per testare l'Observer.

## Logica Filosofica del Controllo

L'errore non è trattato come una semplice differenza di posizione, ma come la deviazione tra l'**intento dinamico dell'utente** e la **realtà inerziale**. Il sistema tende asintoticamente verso il comando utente, "indurendo" la resistenza alle forze esterne senza sacrificare la fluidità dei movimenti.

## Esecuzione Simulazione

```bash
pip install numpy matplotlib
python simulation/pid_sim.py
```
Guarda `simulation_result.png` per vedere come il drone reagisce alle raffiche di vento mantenendo il pitch impostato.
