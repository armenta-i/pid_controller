# High-Integrity Altitude Control System (SITL)

## Project Overview
This project is a **Software-In-The-Loop (SITL)** simulation of a drone altitude control system. It consists of a physical drone simulator and a PID (Proportional-Integral-Derivative) flight controller designed according to the **Joint Strike Fighter (JSF) Air Vehicle C++ Coding Standards**.

The simulation demonstrates how a discrete-time control algorithm can manage the complex physics of vertical flight, including gravity compensation and motor saturation.

[Image of PID control system block diagram with feedback loop]

## Technical Architecture
The system follows strict **Separation of Concerns (SoC)** by isolating physics from logic:
1.  **Drone Simulator (`Drone` class):** Models vertical flight physics using Euler integration. It accounts for gravity ($9.81 m/s^2$), motor thrust acceleration ($20.0 m/s^2$ max), and enforces ground-level collision boundaries ($0m$).
2.  **Altitude Controller (`AltitudeController` class):** A discrete-time PID controller that calculates necessary thrust (0.0 to 1.0) to reach and maintain a target altitude.

## JSF AV C++ Standard Compliance
This project was developed under the constraints of safety-critical embedded systems (Lockheed Martin JSF AV C++):

* **Fixed-Width Types:** Used `float32_t` and `int32_t` (via `<cstdint>`) to ensure deterministic memory usage across hardware architectures.
* **Encapsulation:** All state variables (altitude, velocity, gains) are `private`. Interaction occurs strictly through validated public interfaces.
* **No Dynamic Memory:** All objects are stack-allocated to prevent heap fragmentation and non-deterministic execution timing.
* **Safety Constraints:** Input parameters are validated via a custom `Status` enum. Motor thrust is strictly clamped between 0.0 (off) and 1.0 (max) to prevent mathematical overflow and hardware damage.

## Control Theory & Implementation
### 1. PID Logic
* **Proportional (P):** Corrects immediate error.
* **Integral (I) with Anti-Windup:** Eliminates steady-state error. Includes **Anti-Windup Clamping** to prevent the integral term from growing to infinity while the drone is on the ground.
* **Derivative (D) with Initialization Protection:** Calculates the rate of climb to prevent overshoot. Logic was implemented to baseline the `previous_error_` to prevent "Tick 0" thrust spikes.

[Image of PID controller response curves: P, PI, and PID]

### 2. Physics Simulation
The simulator uses discrete-time integration every $10ms$ ($dt = 0.01$):
* **Acceleration:** $a = (Thrust \cdot MaxAccel) - Gravity$
* **Velocity:** $v_{new} = v_{old} + (a \cdot dt)$
* **Position:** $p_{new} = p_{old} + (v_{new} \cdot dt)$

## Compilation & Usage
To mimic an aerospace production environment, compile with strict safety flags:

```bash
g++ -std=c++11 -Wall -Wextra -Werror -fno-exceptions -fno-rtti \
    main.cpp altitude_controller.cpp drone_simulator.cpp -o drone_sim

    