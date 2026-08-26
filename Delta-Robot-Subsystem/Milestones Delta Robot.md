# Milestones: Delta Robot Subsystem in Webots

This document defines the development milestones for integrating the 3-DOF parallel delta robot into the Webots simulation environment. It matches the structure and style of the navigation subsystem milestones, providing a clear path from basic kinematic integration to publishable research results.

---

## Milestone A — Kinematics and Coordinate Frame Mapping
**Why this matters**: In pure Python, we solved kinematics in an isolated local coordinate system. In Webots, the robot is part of a larger 3D scene. If the coordinate frames, offsets, or units (meters vs. millimeters) are misaligned, the robot will miss targets or command joints outside their physical limits.

### Steps:
1. **Choose the Modeling Representation**: Decide between a physically-resolved parallel model (Option 1: using hinge joints and distance links) or a kinematically-mapped gantry slider (Option 2: using X/Y/Z linear joints driven by the analytical IK solver).
2. **Mount the Subsystem**: Position the robot base centrally beneath the mobile platform chassis in the Webots Scene Tree (`WEEDBOTPRO_MASTER.wbt`).
3. **Write the Webots Interface**: In the Python controller, load `delta_robot_kinematics.py` and implement a mapping function:
   $$\mathbf{P}_{\text{webots}} = \mathbf{R} \cdot \mathbf{P}_{\text{robot}} + \mathbf{T}$$
   where $\mathbf{R}$ is the rotation matrix and $\mathbf{T}$ is the translation vector matching the arm's physical mount point.
4. **Run a Static Calibration Check**: Command the end-effector to touch 5 reference points in Webots. Read the end-effector position using a GPS sensor node or Supervisor field in the controller and compare it with the analytical forward kinematics.

### Done when:
- The coordinate translation is verified, and the absolute position error between the analytical model and the Webots simulation is $< 1.0\text{ mm}$ for all 5 reference points.

---

## Milestone B — Dynamic Joint Control and Step Response (→ Figure 3b)
**Why this matters**: The pure-Python model simulated joint dynamics using a simple second-order differential equation. In Webots, the joints have real mass, link inertia, gearbox friction, and torque limits. We must tune the controllers in Webots to ensure the joints can settle at setpoints within the cycle-time budget.

### Steps:
1. **Configure Actuator Physics**: Set the Webots Motor nodes' torque limits and maximum velocities to match the physical MG996R servo datasheet parameters (torque limit: $0.92\text{ N}\cdot\text{m}$, speed limit: $6.16\text{ rad/s}$).
2. **Implement PID Control**: Write a joint-space PID loop in the controller:
   $$\tau(t) = K_p e(t) + K_i \int e(t)dt + K_d \frac{de(t)}{dt}$$
   and clamp the command to the torque limit.
3. **Log the Step Response**: Command Joint 1 to step from $0^\circ$ to $45^\circ$. Log the timestamp and actual joint angle at every physics step ($32\text{ ms}$) to `pid_response_webots.csv`.
4. **Plot and Analyze**: Create a matplotlib script to plot the step-response curve and compute the rise time, settling time, and percentage overshoot.

### Done when:
- You have a saved figure (`figure_3b_webots_pid.png`) showing the joint settling at the $45^\circ$ setpoint within a $0.5\text{ s}$ window, with a settling time $< 0.4\text{ s}$ and overshoot $< 5.0\%$.

---

## Milestone C — Excision Sequence and Path Optimization (→ Figure 3a)
**Why this matters**: Agriculture fields contain clumps of weeds. If the robot visits weeds in a random detection order, it wastes time and battery power. We must verify that the nearest-neighbor greedy path and the exact Traveling Salesperson Problem (TSP) solver translate to physical savings in the simulator.

### Steps:
1. **Define the Excision Cycle**: Program the 3D trajectory sequence for a target:
   - **Approach**: Move horizontally at $80\text{ mm/s}$ to $50\text{ mm}$ above the weed.
   - **Plunge**: Descend vertically at $40\text{ mm/s}$ to the cutting depth (e.g., $-170\text{ mm}$) while spinning the cutting blade.
   - **Dwell**: Wait $0.3\text{ s}$ to simulate cutting.
   - **Retract**: Ascend vertically back to the clearance height ($-50\text{ mm}$).
2. **Spawn Mock Weed Patches**: Write a Supervisor function that spawns 5–8 weed models at random reachable positions inside the camera frame.
3. **Execute and Log**: Run the same weed layout under three sequencing modes: (1) unsorted detection order, (2) greedy nearest-neighbor, and (3) exact TSP tour. For each run, log the total time and joint travel distances.

### Done when:
- The arm completes physical weeding trials in Webots for 25 randomized scenarios. The results are logged to `path_optimization_webots.csv`, confirming a $> 20\%$ reduction in travel path length for optimized sequencing.

---

## Milestone D — Stop-and-Cut vs. Continuous-Motion (→ New Research Contribution)
**Why this matters**: This is the core research contribution. In most existing systems, the mobile base must stop to let the arm weed. If the mobile base can keep moving and the delta arm tracks weeds *on the fly*, throughput increases dramatically. We will compare both strategies to generate publishable comparative data.

### Steps:
1. **Implement Stop-and-Cut**: The mobile base drives, stops when weeds are detected, waits for the delta arm to execute its optimized cut sequence, and then resumes navigation.
2. **Implement Continuous Tracking**: The base drives at a constant speed ($0.1\text{ to }0.3\text{ m/s}$). The delta controller transforms the weed coordinates dynamically by subtracting the base displacement in real-time, allowing the arm to track and plunge on the moving target.
3. **Measure Performance Metrics**: Record the total weeding time for a $10\text{ m}$ row, the success rate (successful cuts inside a $5\text{ mm}$ tolerance), and a proxy for energy consumption (sum of wheel motor and joint actuator power over time).

### Done when:
- You have a comparative CSV (`coordination_comparison.csv`) and a grouped bar chart plotting throughput (weeds/min), energy per weed, and success rate for both strategies across a range of speeds.

---

## Milestone E — Density-Adaptive Speed Control (→ New Novel Algorithm)
**Why this matters**: Continuous tracking fails if the local weed density is too high—the arm runs out of workspace before it can cut them all. We propose a novel algorithm: **Density-Adaptive Speed Control (DASC)**. The mobile base speed scales dynamically based on lookahead weed density: slowing down in dense patches and speeding up in clean rows.

### Steps:
1. **Implement lookahead queueing**: Use the camera footprint to count upcoming weeds and estimate queue size.
2. **Scale speed dynamically**: Define a control law for base velocity:
   $$v_{\text{base}}(t) = v_{\text{max}} \cdot f(N_{\text{active\_weeds}})$$
   where the base slows down or temporarily stops only if the queue exceeds the arm's physical processing speed.
3. **Run Comparative Field Scenarios**: Simulate a $10\text{ m}$ row with mixed weed densities (sparse zones, dense patches). Compare DASC against constant-speed continuous tracking and standard stop-and-cut.

### Done when:
- The simulation results demonstrate that DASC achieves a weeding efficiency of $>95\%$ in dense zones while maintaining a higher average travel speed and lower energy footprint than stop-and-cut. These figures and tables will form the core of the results section in the final paper.
