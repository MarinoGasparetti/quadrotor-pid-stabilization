# Quadrotor Stabilization Control System (QSCS)
## Advanced Multi-Rate PID & Active Disturbance Rejection Architecture

This repository contains a high-fidelity control stack for multi-rotor UAVs, specifically engineered to decouple attitude stabilization from trajectory maintenance under non-deterministic environmental loads (e.g., wind gusts).

### 1. Control Architecture Overview

The system is designed around a **Multi-Rate Concurrent Task Scheduler**, separating time-critical stabilization from high-level navigation logic.

#### A. Inner Loop: Attitude Stabilization (400Hz - 1kHz)
*   **Target**: Zero-latency response to angular rate deviations.
*   **Logic**: High-frequency PID loop with integrated Gain Scheduling.

#### B. Outer Loop: Trajectory & Disturbance Observation (50Hz - 100Hz)
*   **Target**: Environmental force estimation and setpoint correction.
*   **Logic**: Implements an **Advanced Disturbance Observer** (DOB).

### 2. Performance Benchmarks

Validated against a standard PID controller under realistic conditions: step wind gust (force=15, t=4s–6s) + gaussian measurement noise (σ=0.02).

| Metric | Standard PID | Advanced DOB | Improvement |
|--------|--------------|--------------|-------------|
| **IAE (Integral Absolute Error)** | 147.43 | 117.09 | **+20.58%** |
| **Threshold (CI gate)** | — | — | > 15% required |
| **Test Result** | — | — | **PASSED** |

> The Advanced DOB maintains a consistent 20%+ IAE advantage over standard PID under step disturbance. The CI/CD pipeline enforces a minimum 15% improvement threshold on every commit.

### 3. Wind Rejection Simulation

The `simulation/pid_sim.py` runs a 1D pitch dynamics model with a wind gust injected between t=4s and t=6s. The Disturbance Observer corrects in real-time, keeping the drone on the 20° pitch setpoint with bounded correction effort (clamped to ±20).

### 4. Implementation Details (C++)

*   **`core/pid_controller.hpp`**: Thread-safe dual-profile PID with anti-windup (conditional integration) and output clamping.
*   **`core/disturbance_observer.hpp`**: Observer with initialization guard (no spike at t=0) and correction clamping. Computes counter-torque from delta between expected and measured acceleration.
*   **`core/scheduler.hpp`**: Priority-based task orchestrator using a dedicated thread pool to ensure consistent cycle times (dt) across control layers.

### 5. Running Simulations

```bash
# Wind rejection simulation → simulation/wind_rejection_result.png
python simulation/pid_sim.py

# PID vs DOB benchmark → benchmarking/benchmark_results.png
python benchmarking/compare.py
```

### 6. CI/CD

Validated via GitHub Actions on every push:
*   **Quadrotor CI**: Compiles C++ core and runs unit tests.
*   **Simulation CI**: Validates basic flight stability.
*   **Performance Benchmark**: Enforces DOB improvement > 15% over standard PID. Fails build if threshold not met.

---
**Lead Engineer:** Marino Gasparetti  
**Status:** Validated in Simulation — Benchmark passed (IAE improvement: +20.58%)
