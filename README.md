# 🤖 Black Rose — Differential Drive Line Follower Robot

![Arduino](https://img.shields.io/badge/Platform-Arduino%20Uno-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![MATLAB](https://img.shields.io/badge/Simulation-MATLAB%20R2024b-0076A8?style=for-the-badge&logo=mathworks&logoColor=white)
![SolidWorks](https://img.shields.io/badge/CAD-SolidWorks%202022-FF0000?style=for-the-badge&logoColor=white)
![Proteus](https://img.shields.io/badge/Circuit-Proteus%208-1BA0D7?style=for-the-badge&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)

A fully engineered line-following robot built from the ground up, covering mechanical design, embedded control, circuit simulation, kinematic modeling, and MATLAB simulation. Developed as a complete systems engineering project for the IEEE Line Following Robot Competition.

---

## 📋 Table of Contents

- [Project Overview](#project-overview)
- [Repository Structure](#repository-structure)
- [Hardware & Components](#hardware--components)
- [Embedded System (Arduino)](#embedded-system-arduino)
- [Circuit Design (Proteus)](#circuit-design-proteus)
- [Mechanical Design (SolidWorks)](#mechanical-design-solidworks)
- [Kinematics & Control Theory](#kinematics--control-theory)
- [MATLAB Simulation](#matlab-simulation)
- [Getting Started](#getting-started)
- [Results](#results)
- [Team](#team)

---

## Project Overview

**Black Rose** is a differential drive robot designed to follow a black line on a white surface using 5 IR sensors and a PD controller. The project spans the full engineering stack:

- **Mechanical**: 3D modeled chassis in SolidWorks with two DC motors, a caster wheel, and a sensor mount
- **Electrical**: Arduino Uno + L298N motor driver + 5× IR sensors, simulated in Proteus 8
- **Embedded**: PD controller with sensor-guided turn detection (90°, U-turns, lost recovery)
- **Mathematical**: Full differential drive kinematic model (forward, inverse, odometry)
- **Simulation**: MATLAB simulation with telemetry showing speed, heading, filtered error, and lateral deviation

---

## Repository Structure

```
Black-Rose-Line-Follower/
│
├── embedded/
│   ├── line-follower.ino                 # Arduino PD controller
│   └── schematic_screenshot.png          # Proteus circuit photo
│
├── kinematics/
│   ├── Kinematics_final_version.rar      # Kinematics scripts/files
│   └── Robot_Kinematics_Guide.pdf        # Full kinematic derivation
│
├── mechanical/
│   ├── solidworks_top_view.png           # SolidWorks top view
│   ├── solidworks_bottom_view.png        # SolidWorks bottom view
│   ├── solidworks_side_view_1.png        # SolidWorks side view 1
│   ├── solidworks_side_view_2.png        # SolidWorks side view 2
│   ├── tack_2.m                          # MATLAB simulation script
│   └── matlab_telemetry.png             # MATLAB telemetry screenshot
│
├── docs/
│   └── IEEE_LFR_Final.docx              # IEEE-format project report
│
└── README.md
```

---

## Hardware & Components

| Component | Model | Quantity |
|-----------|-------|----------|
| Microcontroller | Arduino Uno (ATmega328P) | 1 |
| Motor Driver | L298N (H-Bridge) | 1 |
| IR Sensors | TCRT5000 / JoiNMax IR Modules | 5 |
| DC Motors | TT Gear Motor | 2 |
| Caster Wheel | Ball Caster | 1 |
| Power Supply | AA Battery Pack | 1 |
| Buzzer | Passive 5V Buzzer | 1 |
| LED | Status LED | 1 |
| Chassis | Custom SolidWorks-designed car chassis | 1 |

### Pin Mapping

| Arduino Pin | Connected To |
|-------------|-------------|
| A0–A4 | IR Sensors S4–S0 (right to left) |
| D5 (PWM) | ENA — Right Motor speed |
| D6 (PWM) | ENB — Left Motor speed |
| D8, D9 | IN1, IN2 — Right Motor direction |
| D10, D11 | IN3, IN4 — Left Motor direction |
| D13 | Buzzer |
| D7 | Status LED |

---

## Embedded System (Arduino)

**File:** `embedded/line-follower.ino`

### Algorithm Overview

The main loop runs a PD controller for straight-line following and switches to sensor-guided turning for junctions.

#### Sensor Reading & Error Computation

Five IR sensors (S0–S4) are assigned weights `{-2, -1, 0, 1, 2}`. The weighted average gives a signed lateral error:

```
error = Σ(weight[i] × sensor[i]) / Σ(sensor[i])
```

#### PD Controller

```
correction = Kp × error + Kd × (error − lastError)
Left  motor speed = BASE_SPEED + correction
Right motor speed = BASE_SPEED − correction
```

**Tuned constants:**

| Parameter | Value |
|-----------|-------|
| `BASE_SPEED` | 130 |
| `MAX_SPEED` | 200 |
| `TURN_SPEED` | 150 |
| `Kp` | 35.0 |
| `Kd` | 20.0 |

#### Turn Detection Logic

| Condition | Trigger | Action |
|-----------|---------|--------|
| **U-Turn** | All 5 sensors active | Spin right until center re-acquires line |
| **90° Right** | Rightmost active, leftmost clear | Spin right until center re-acquires line |
| **90° Left** | Leftmost active, rightmost clear | Spin left until center re-acquires line |
| **Lost** | No sensors active | Reverse slowly at speed 80 |

All turns use `spinUntilLine()` — a sensor-guided spin with a 1.5s safety timeout. This eliminates delay-based timing and adapts automatically to any junction geometry.

---

## Circuit Design (Proteus)

**File:** `embedded/schematic_screenshot.png`  
**Tool:** Proteus 8 Professional

![Proteus Schematic](embedded/schematic_screenshot.png)

The schematic captures the full system:
- Arduino Uno as the central controller
- L298N motor driver connected to two DC motors
- 5× IR obstacle sensor modules (TCRT5000-compatible)
- Buzzer and LED for status indication
- Battery supply with protection for motor transients

The schematic was validated in Proteus simulation before hardware assembly.

---

## Mechanical Design (SolidWorks)

**Tool:** SOLIDWORKS Premium 2022 SP1.0  
**Files:** `mechanical/solidworks_*.png`

### Assembly Breakdown

| Part | Description |
|------|-------------|
| Car Chassis | Main body plate with motor mounts, sensor rail, and caster pocket |
| Robot Smart Car Kit v2 | Drive wheel sub-assembly (2× TT motors + yellow spoke wheels) |
| Battery Box | AA cell holder mounted on the upper deck |
| Blackboard v2.0 | Arduino Uno board |
| L298N Module | Motor driver mounted on chassis |
| 15× Pan-head screws | Full fastener constraints (Concentric + Coincident mates) |

### Views

| Top View | Bottom View |
|----------|-------------|
| ![Top](mechanical/solidworks_top_view.png) | ![Bottom](mechanical/solidworks_bottom_view.png) |

| Side View 1 | Side View 2 |
|-------------|-------------|
| ![Side 1](mechanical/solidworks_side_view_1.png) | ![Side 2](mechanical/solidworks_side_view_2.png) |

---

## Kinematics & Control Theory

**Files:** `kinematics/Robot_Kinematics_Guide.pdf` · `kinematics/Kinematics_final_version.rar`

### System Parameters

| Symbol | Description |
|--------|-------------|
| `r` | Wheel radius |
| `L` (= 2b) | Wheelbase / Track width |
| `ωR`, `ωL` | Right and left wheel angular velocities (rad/s) |
| `v` | Linear velocity of robot center (m/s) |
| `ω` | Angular velocity of robot body (rad/s) |
| `θ` | Heading in global frame |

### Forward Kinematics

Maps wheel velocities → body velocities:

```
[v]   [r/2   r/2 ] [ωR]
[ω] = [r/L  -r/L ] [ωL]
```

### Unicycle Model (Global Frame)

```
[ẋ ]   [cos(θ)  0] [v]
[ẏ ] = [sin(θ)  0] [ω]
[θ̇]   [0       1]
```

### Odometry Integration

```
P(t) = P(t−Δt) + Δt · J(θ) · [ωR, ωL]ᵀ
```

### Inverse Kinematics

```
[ωR]   (1/r) [1   L/2] [v]
[ωL] =       [1  -L/2] [ω]
```

### PID Controller

```
ω(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·(de/dt)
```

> **Implementation Note:** The current implementation uses PD only (no I term), which is sufficient for this track geometry. On microcontrollers, use timer interrupts to keep Δt constant and avoid odometry drift.

---

## MATLAB Simulation

**Files:** `mechanical/tack_2.m` · `mechanical/matlab_telemetry.png`  
**Tool:** MATLAB R2024b

![MATLAB Telemetry](mechanical/matlab_telemetry.png)

The simulation models the full robot on the Black Rose rectangular track and produces a real-time telemetry dashboard:

| Plot | Description |
|------|-------------|
| **Speed (m/s)** | Linear velocity over time — shows acceleration and turning slowdowns |
| **Heading (rad)** | Orientation in global frame — sharp spikes indicate 90° / U-turns |
| **Filtered Error** | Lateral deviation from line center after low-pass filtering |
| **Deviation (m)** | Absolute position error — peaks at corners, near-zero on straights |
| **Sensor Readings** | Bar chart of 5 sensor distances to line (L2, L1, C, R1, R2) |

The dashboard includes **Restart** and **Reverse Direction** interactive controls.

---

## Getting Started

### 1. Upload Arduino Code

```bash
# Open in Arduino IDE
File > Open > embedded/line-follower.ino

# Select board: Arduino Uno
# Select correct COM port
# Upload
```

### 2. View Circuit Schematic

```
Open embedded/schematic_screenshot.png
```

### 3. Run MATLAB Simulation

```matlab
% In MATLAB R2024b or later
cd mechanical/
run('tack_2.m')
% Two figures open: track animation + telemetry dashboard
```

### 4. Open Kinematics Files

```
Extract kinematics/Kinematics_final_version.rar
Reference kinematics/Robot_Kinematics_Guide.pdf for full theory
```

---

## Results

- ✅ Robot successfully follows a black line including 90° corners and U-turns
- ✅ PD controller eliminates oscillation while maintaining responsive correction
- ✅ Sensor-guided turning adapts automatically to any junction without delay tuning
- ✅ MATLAB telemetry confirms heading stability and low lateral deviation on straights
- ✅ Proteus simulation validated wiring before hardware build
- ✅ SolidWorks assembly provides full mechanical reference for fabrication

---

## Team

**Black Rose Robotics Team**  
Mohamed Sayed Nageb 
Ziad Asem Sadky
Ahmed Sayed Mahmoud
Ahmed Mohamed Ahmed Abdel Haleem
Fatma Khaled Mohamed Kamel
Elham Fadl Ahmed Ahmed
Rokaya Mansour Ibrahim Abdel Moneim
Farah Abdel Wahab Mohamed Abdel Wahab
Fatma Mahmoud Mohamed Sadek
Omnia El-Sayed Youssef zalama
---

## License

MIT License — see `LICENSE` for details.
