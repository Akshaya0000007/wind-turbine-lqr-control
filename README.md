Wind Turbine Speed Control Using LQR

This project is a MATLAB simulation study. No physical turbine hardware was built or tested — all results are simulation-based.

Overview

Wind turbines are exposed to constantly changing wind conditions, which causes rotor speed to deviate from its desired operating point. This project explores using a Linear Quadratic Regulator (LQR) feedback controller to regulate rotor speed under varying wind conditions, implemented and simulated in MATLAB.

Core question: How can rotor speed be controlled so the turbine stays close to its desired operating condition despite fluctuating wind?

Why Feedback Control?

Wind disturbances continuously push rotor speed away from its target. A feedback loop lets the controller observe the current state of the system and calculate an appropriate control action in response, rather than applying a single fixed input regardless of conditions.

Wind Disturbance
      ↓
Wind Turbine Model
      ↓
Rotor Speed
      ↓
State / Feedback Measurement
      ↓
LQR Controller
      ↓
Control Input → back to Wind Turbine
Why LQR

LQR formulates the control problem as an optimization: rather than manually tuning controller gains, it computes a control input that minimizes a quadratic cost function balancing:

State deviation — how far the rotor speed is from the desired value
Control effort — how aggressively the controller intervenes

This is governed by two weighting matrices:

Q — penalizes state error (higher Q → controller prioritizes staying close to the desired state)
R — penalizes control effort (higher R → controller avoids large/aggressive control actions)
Why LQR over PID

PID is a common and effective choice for this class of problem too. LQR was chosen here to explore an optimization-based approach that jointly considers multiple system states and control effort, rather than as a claim that it's categorically superior to PID for this application.

What Was Modeled in MATLAB
The wind turbine represented as a dynamic (state-space) system
Rotor state monitored through a feedback loop
LQR controller design and gain computation
Rotor-speed response simulated under varying wind input conditions
Scope & Boundaries

This project is simulation-only. It includes:

✅ System modeling
✅ LQR controller implementation
✅ Rotor-speed response analysis under simulated disturbance

It does not include:

❌ Physical turbine hardware
❌ Real-world field testing or measurements
Key Concepts

The project centers on three connected ideas: Feedback → State-space modeling → LQR optimization. Understanding how these three link together (system state feeding into an optimal controller that minimizes a cost function) is the core of the project.
