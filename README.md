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

The system performance is automatically validated against a standard PID controller under realistic environmental conditions (wind gusts + measurement noise).

| Metric | Standard PID | Advanced DOB | Improvement |
|--------|--------------|--------------|-------------|
| **IAE (Integral Absolute Error)** | 11.85 | 9.42 | **+20.51%** |
| **Recovery Time (5N gust)** | ~1.8s | ~0.6s | **+66%** |

> **Note**: The Advanced DOB architecture shows significant superiority in noisy environments, maintaining structural stiffness where traditional PID systems begin to oscillate or drift.

### 3. Implementation Details (C++)

*   **`core/pid_controller.hpp`**: Thread-safe implementation of a dual-profile PID. Features anti-windup (conditional integration) and output clamping.
*   **`core/disturbance_observer.hpp`**: Stateless observer that computes the required counter-torque based on the delta between expected and measured acceleration.
*   **`core/scheduler.hpp`**: Priority-based task orchestrator using a dedicated thread pool to ensure consistent cycle times (dt) across different control layers.

### 4. Simulation & Verification

The provided Python suite (`simulation/pid_sim.py`) validates the control law using a 1D/2D dynamical model.
The automated benchmark suite (`benchmarking/compare.py`) provides the quantitative comparison used in the CI/CD pipeline.

```bash
# Run the performance benchmark
python benchmarking/compare.py
```

### 5. Deployment & CI/CD

The system is continuously validated via GitHub Actions:
*   **Quadrotor CI**: Compiles the C++ core and runs unit tests.
*   **Simulation CI**: Validates basic flight stability.
*   **Performance Benchmark**: Ensures the DOB logic maintains a performance lead > 15% over standard PID.

---
**Lead Engineer:** Marino Gasparetti  
**Status:** Validated in Simulation - Performance benchmarks met.
