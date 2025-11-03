# VEX V5 Cosine Motion Profile

<p align="center">
  <img src='imgs/cos.png'>
</p>

This project implements a **Cosine-based motion profiling algorithm** for the VEX V5 robotics system using C++.  
It allows the robot to move a precise distance smoothly and accurately while maintaining a stable heading through an integrated PID controller.

Developed during the **2023–2024 VEX Robotics Competition "Over Under"** season by **Team 11645A – Taiwan**.

---

## Overview

The core feature of this project is the **`Cos_move()`** function — a custom driving algorithm that combines:

- **Cosine (S-curve) motion profiling** for smooth acceleration and deceleration  
- **PID-based heading correction** using the Inertial Sensor  
- **Encoder feedback** from all six drive motors for distance tracking  

This ensures the robot can **drive straight and stop precisely**, minimizing overshoot and drift during autonomous operation.

---

## Visualization
The following plot illustrates the motion profile used by `Cos_move`,  
showing how the cosine curve provides gradual acceleration and deceleration instead of abrupt changes:

🔗**[View the interactive graph on Desmos →](https://www.desmos.com/calculator/z5b7etdhab)**

> The curve \( y(t) = 0.5 \cdot a \cdot (1 - \cos(\pi t / T)) \) defines position over time,  
> producing smooth velocity and acceleration transitions.

## Function Description

### `double Cos_move(double dis, double v, double target, double m)`

| Parameter | Description |
|------------|-------------|
| `dis` | Target distance to move (in linear units) |
| `v` | Nominal linear velocity |
| `target` | Desired heading angle (degrees) |
| `m` | Reserved for future tuning or scaling |

### Core Features
- Converts linear distance into encoder degrees  
- Generates a **cosine velocity curve** over time:  
  \[
  y(t) = 0.5 \times \text{mag} \times (1 - \cos(\pi t / T))
  \]
- Uses **PID control** to maintain heading via Inertial4 sensor:
  \[
  \text{output} = K_p e + K_i \int e + K_d \frac{de}{dt}
  \]
- Applies correction to left and right drive speeds independently  
- Stops the motors with braking once the target distance is reached  

---

---

## Dependencies
- **VEX V5 SDK / VEXcode Pro V5**
- **C++17** or later  
- Hardware:
  - 6 drive motors (L1–L3, R1–R3)
  - Inertial Sensor (`Inertial4`)

---

## PID and Motion Parameters
You can tune the control constants for your specific robot setup:

```cpp
double Kp = 2.2;      // Proportional gain
double Ki = 0.00001;  // Integral gain
double Kd = 0.5;      // Derivative gain
```
and adjust conversion constants for wheel circumference and gear ratio:
```cpp
double mag = dis * 19.339 / 0.8265;  // distance to degrees
```

## Contributors
- Tzu-Huang Liu (Zackery Liu)
- Joseph Lin
- Alex Lee
- Benjamin Wu


