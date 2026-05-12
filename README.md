# Quadrotor PID Stabilization — Technical Architecture

High-performance control system for UAVs, featuring multi-rate task scheduling and active disturbance rejection.

## Technical Specifications

- **Dual-Rate Concurrency**:
  - **Inner Loop (High-Freq, 400Hz+)**: Critical attitude stabilization.
  - **Outer Loop (Low-Freq, 50Hz)**: Navigation, gain scheduling, and disturbance observation.
- **Active Gain Scheduling**: Automated transition between `Static` (high-precision hovering) and `Dynamic` (high-responsiveness) PID profiles.
- **Disturbance Observer (ADRC Lite)**: Implements error marginal derivative analysis to isolate environmental forces (wind) from user intent.
- **Trajectory Integrity**: Corrects the PID setpoint based on the delta between commanded pitch and inertial response, ensuring trajectory maintenance under external load.

## Project Structure

- `core/` (C++11/17)
  - `pid_controller.hpp`: Dual-profile PID implementation.
  - `disturbance_observer.hpp`: Wind rejection and trajectory correction logic.
  - `scheduler.hpp`: Multi-threaded task orchestrator.
- `simulation/` (Python 3)
  - `pid_sim.py`: Numerical validation of wind rejection algorithms.

## Operational Logic

The system defines the control error as the deviation between the **user-commanded dynamic vector** and the **actual inertial state**. By applying a derivative correction to the command margin, the observer predicts and counteracts external forces before they translate into significant positional drift.

## Simulation & Validation

```bash
pip install numpy matplotlib
python simulation/pid_sim.py
```
Output: `simulation_result.png` displays the performance delta between standard PID and the Active Disturbance Rejection layer.
