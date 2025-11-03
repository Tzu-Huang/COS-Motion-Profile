# 🚀 VEX V5 Cosine Motion Drive

This project implements a **cosine-based motion profiling algorithm** for the VEX V5 robotics system using C++.  
It allows the robot to move a precise distance smoothly and accurately while maintaining a stable heading through an integrated PID controller.

Developed during the **2023–2024 VEX Robotics Competition "Over Under"** season by **Team 11645A – Taiwan**.

---

## 🧠 Overview

The core feature of this project is the **`Cos_move()`** function — a custom driving algorithm that combines:

- **Cosine (S-curve) motion profiling** for smooth acceleration and deceleration  
- **PID-based heading correction** using the Inertial Sensor  
- **Encoder feedback** from all six drive motors for distance tracking  

This ensures the robot can **drive straight and stop precisely**, minimizing overshoot and drift during autonomous operation.

---

## ⚙️ Function Description

### `double Cos_move(double dis, double v, double target, double m)`

| Parameter | Description |
|------------|-------------|
| `dis` | Target distance to move (in linear units) |
| `v` | Nominal linear velocity |
| `target` | Desired heading angle (degrees) |
| `m` | Reserved for future tuning or scaling |

### 🧩 Core Features
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

## 🛠️ Project Structure

