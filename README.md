# Quadrotor Stabilization Control System (QSCS)
## Advanced Multi-Rate PID & Active Disturbance Rejection Architecture

This repository contains a high-fidelity control stack for multi-rotor UAVs, specifically engineered to decouple attitude stabilization from trajectory maintenance under non-deterministic environmental loads (e.g., wind gusts).

### 1. Control Architecture Overview

The system is designed around a **Multi-Rate Concurrent Task Scheduler**, separating time-critical stabilization from high-level navigation logic.

#### A. Inner Loop: Attitude Stabilization (400Hz - 1kHz)
*   **Target**: Zero-latency response to angular rate deviations.
*   **Logic**: High-frequency PID loop with integrated Gain Scheduling.
*   **Gain Scheduling Profiles**:
    *   `Static Profile`: Optimized for positional stiffness and high-frequency noise rejection during hovering.
    *   `Dynamic Profile`: Optimized for command tracking and reduced damping during high-velocity maneuvers.

#### B. Outer Loop: Trajectory & Disturbance Observation (50Hz - 100Hz)
*   **Target**: Environmental force estimation and setpoint correction.
*   **Logic**: Implements an **Active Disturbance Observer** (ADO) that monitors the "Marginal Error Derivative" between the commanded vector and the measured inertial response.

### 2. Technical Specifications: Wind Rejection Logic

The core innovation lies in the treatment of the error term. Instead of a standard positional error, the system calculates a **Trajectory Tracking Error (TTE)**:

$$ E_{TTE} = \int (\text{User Intent} - \text{Inertial Response}) dt $$

The `DisturbanceObserver` applies a derivative gain to this margin to detect external perturbations (force vectors not originating from motor thrust). 
*   **Predictive Correction**: The observer injects an additive offset into the Inner Loop's setpoint.
*   **Asymptotic Convergence**: The correction force is modulated to ensure the drone's attitude tends asymptotically toward the user's intended pitch, effectively "stiffening" the drone's resistance to wind without causing derivative kick or control surface saturation.

### 3. Implementation Details (C++)

*   **`core/pid_controller.hpp`**: Thread-safe implementation of a dual-profile PID. Features anti-windup (conditional integration) and output clamping.
*   **`core/disturbance_observer.hpp`**: Stateless observer that computes the required counter-torque based on the delta between expected and measured acceleration.
*   **`core/scheduler.hpp`**: Priority-based task orchestrator using a dedicated thread pool to ensure consistent cycle times (dt) across different control layers.

### 4. Simulation & Verification

The provided Python suite (`simulation/pid_sim.py`) validates the control law using a 1D/2D dynamical model:
1.  **Steady State**: Hovering stability at $t < 3s$.
2.  **Transient Response**: Step input response to user command.
3.  **Disturbance Rejection**: Real-time compensation of a constant 5N wind force and impulsive gusts.

```bash
# Requirements: Python 3.x, NumPy, Matplotlib
python simulation/pid_sim.py
```

### 5. Deployment & CI/CD

The system is continuously validated via GitHub Actions. Every commit triggers a simulation run to ensure that changes to the PID constants or the observer's alpha-gain do not introduce oscillatory behavior or exceed the defined stability margins.

---
**Lead Engineer:** Marino Gasparetti  
**Status:** Validated in Simulation - Ready for Hardware-in-the-Loop (HIL) testing.
