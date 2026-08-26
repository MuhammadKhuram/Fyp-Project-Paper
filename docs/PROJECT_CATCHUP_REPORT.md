# Subsystem Review: Delta Robot & Navigation Subsystems

This report documents the planned proposals, implemented solutions, and current validation results for the Delta Robot and Navigation Subsystems of the Autonomous Weed Removal Robot (AWRR).

---

## 1. Delta Robot Subsystem

### Kinematics & Workspace
* **Proposed in Paper**: Analytically derive forward and inverse kinematics (FK/IK) and numerically map the 3D reachable workspace limits.
* **What We Did**: Formulated closed-form geometric kinematics equations and mapped the operating reach by exhaustive sampling up to a 50×50×50 grid.
* **Current Results**:
  * **Kinematic consistency confirmed**: Maximum round-trip position error is \(2.49 \times 10^{-13}\) mm, confirming zero mathematical drift.
  * **Workspace radial boundaries mapped**: Reachable radius converges to **208.0 mm** with joint limits enforced (and **253.1 mm** when limits are neglected).

### Trajectory Planning & Actuator Control
* **Proposed in Paper**: Sequence visits to multiple detected weeds using nearest-neighbor logic and control joint angles with stable PID responses.
* **What We Did**: Coded a greedy nearest-neighbor planner, represented motor dynamics (inertia and damping), and manually tuned joint PID values.
* **Current Results**:
  * **Path reduction achieved**: Nearest-neighbor routing saves **23.4%** path length vs. unsorted sequence, with only **5.6%** overhead vs. optimal TSP.
  * **Rapid settling achieved**: PID joint controller (\(K_p=0.08, K_i=0.02, K_d=0.008\)) settles targets in **under 0.5 s** with minimal overshoot.

### Performance Robustness
* **Proposed in Paper**: Assess kinematic and trajectory planning robustness under varying weed coordinates and densities.
* **What We Did**: Simulated the complete weed-removal pipeline across 25 independent randomized scenarios (3–8 weeds per scenario).
* **Current Results**:
  * **Reachability verified**: **85.0%** of randomly generated weeds are reachable without base repositioning.
  * **Stable operational time**: Mean scenario cycle time converges to **3.10 s** (standard deviation = 0.99 s).
  * **Optimality verified**: Greedy heuristic path length remains within **3.6%** of exact optimal TSP solutions on average.

---

## 2. Navigation Subsystem

### Serpentine Coverage & Sensor Fusion
* **Proposed in Paper**: Traverse the field systematically using serpentine coverage planning and track coordinates using EKF sensor fusion.
* **What We Did**: Built closed-loop waypoint following, programmed an EKF fusing noisy GPS with IMU, and simulated canopy GPS blackouts.
* **Current Results**:
  * **Systematic coverage complete**: Completed 28-waypoint passes on a 10m × 10m field with closed-loop steering and speed tapering.
  * **Blackout dead reckoning**: EKF bounded drift during a 30 s GPS outage (max error \(\sim 0.2\) m) and recovered to **<0.05 m** error when the GPS signal resumed.

### Row Switching (Crab-Walk vs. Turn)
* **Proposed in Paper**: Incorporate holonomic transitions to reduce row-switch duration, eliminate alignment delays, and prevent crop damage.
* **What We Did**: Automated holonomic crab-walking vs. 180° turn-in-place at row ends and benchmarked duration and velocity proxy energy.
* **Current Results**:
  * **Significant speedup**: Lateral crab-walking completed switches in **2.4 s** vs. **7.8 s** for turning-in-place (a **69% reduction**).
  * **Drastic energy savings**: Crab-walking reduced overall wheel commands by **60%** (integrated speed proxy of 12.8 vs. 32.1 units).
  * **Soil/Crop preservation**: Eliminated in-place wheel rotations, avoiding tire shear stresses in crop root zones.

---

## 3. Webots Simulation Methodology & Future Proposals

### Webots Simulation (What We Did)
* **Environment Setup**: Programmed a high-fidelity 3D agricultural field simulation in Webots, replicating a 10m × 10m operating space with organized crop rows and weed distributions.
* **Sensor & Control Modeling**: Modeled active GPS and IMU physical nodes, closed-loop waypoint-following controllers, and automated crab-walking and turn-in-place transition kinematics.

### Empirical Data Collection (How Results Were Collected)
* **High-Frequency Logging**: Captured coordinate and orientation states directly from Webots at 32 ms steps.
* **Filter Robustness Tests**: Injected Gaussian sensor noise (\(\sigma = 0.02 - 0.05\) m) and programmed a 30-second GPS blackout to verify the EKF's ability to maintain tracking via dead reckoning and stabilize when signals resumed.
* **Actuator Energy Proxy**: Integrated absolute wheel speed commands over the duration of the row-transition window to benchmark motor energy usage.

### Future Proposals & Comparative Benchmarks
* **Delta vs. Serial Link Arm Simulation**: Mount both the 3-DOF parallel Delta arm model and a 6-DOF serial link arm model on the moving Webots platform.
* **Dynamic Disturbance Analysis**: Subject both arm systems to identical platform acceleration/deceleration and soil-induced vibration disturbances while traversing crop rows.
* **Benchmarking Metrics**:
  * **Dynamic Tracking Latency**: Measure end-effector settling time and spatial overshoot when descending to cut weeds while the base is moving.
  * **Actuator Power Draw**: Record active electrical current, joint torque saturation, and mechanical load profiles.
  * **Soil & Platform Stability**: Evaluate how parallel vs. serial weight profiles shift the platform center of mass and affect wheel slip and soil shear stress.
