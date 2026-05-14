# Quadrotor Stabilization Control System (QSCS)
## Advanced Multi-Rate PID & Active Disturbance Rejection Architecture

This repository contains a high-fidelity control stack for multi-rotor UAVs, specifically engineered to decouple attitude stabilization from trajectory maintenance under non-deterministic environmental loads (e.g., wind gusts) and aerodynamic "Aerial Rebound" effects.

### 1. Control Architecture Overview

The system is designed around a **Multi-Rate Concurrent Task Scheduler**, separating time-critical stabilization from high-level navigation logic.

#### A. Inner Loop: Attitude Stabilization (400Hz - 1kHz)
*   **Target**: Zero-latency response to angular rate deviations.
*   **Logic**: High-frequency PID loop with integrated Gain Scheduling.

#### B. Outer Loop: Trajectory & Disturbance Observation (50Hz - 100Hz)
*   **Target**: Environmental force estimation and setpoint correction.
*   **Logic**: Implements an **Advanced Disturbance Observer** (DOB) with an **Adaptive Active Reset Layer**.

### 2. Key Innovation: Adaptive Reset & Evaluation Layer
To solve the **"Aerial Rebound"** (overshoot caused by estimation lag during sudden gust cessation), we introduced a dynamic evaluation layer:
*   **Active Evaluation**: Analyzes the coherence between PID intent and Observer estimation.
*   **Mediated Adaptive Reset**: Instead of a hard reset, the system calculates a decay factor based on the system's kinetic energy at the setpoint crossing.
*   **Energy-Based Decay**: `Reset_Factor = exp(-Energy * k_calib)`. This ensures maximum precision during slow approach and maximum safety during high-velocity recovery.

### 3. Performance Benchmarks

Validated against a standard PID controller under realistic conditions: high-inertia dynamics (0.8), coupled Pitch/Roll axes, and sudden step wind gusts.

| Metric | Standard PID | Adaptive DOB (Fixed) | Result |
|--------|--------------|----------------------|-------------|
| **Max Overshoot (Peak)** | ~55.0° | **29.56°** | **-46% Overshoot** |
| **Stability (Aerial Rebound)** | High Oscillations | **Damped / Mediated** | **PASSED** |
| **T=0 Spike Initialization** | Present (Legacy) | **Eliminated (Lazy Init)** | **FIXED** |

> The Adaptive DOB architecture eliminates the 200%+ divergence risk of standard DOBs by using energetic-mediated resets, outperforming PID by damping the recovery phase effectively.

### 4. Implementation Details (C++)

*   **`core/pid_controller.hpp`**: Thread-safe dual-profile PID with anti-windup and output clamping.
*   **`core/disturbance_observer.hpp`**: Observer with adaptive active reset logic (integrated with Evaluation Layer).
*   **`core/scheduler.hpp`**: Priority-based task orchestrator for multi-rate execution.

### 5. Running Simulations

```bash
# Coupled Pitch/Roll simulation → simulation/coupled_rebound_sim.png
python simulation/pid_sim.py

# Adaptive Benchmark (with active reset) → benchmarking/benchmark_rebound_fixed.png
python benchmarking/compare.py
```

### 6. CI/CD

Validated via GitHub Actions on every push:
*   **Quadrotor CI**: Compiles C++ core.
*   **Performance Benchmark**: Verifies that the Active Evaluation Layer keeps overshoot within safe bounds (<35° in peak).

---
**Lead Engineer:** Marino Gasparetti  
**Status:** Validated - Adaptive Reset Layer Implemented.
