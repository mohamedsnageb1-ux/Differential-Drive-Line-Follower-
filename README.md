# 🤖 Black Rose — Differential Drive Line Follower Robot

A fully engineered line-following robot built from the ground up, covering mechanical design, embedded control, circuit simulation, kinematic modeling, and MATLAB simulation. Developed as a complete systems engineering project.

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
- **Simulation**: MATLAB/Simulink-based telemetry showing speed, heading, filtered error, and lateral deviation

---

## Repository Structure

```
Black-Rose-Line-Follower/
│
├── embedded/
│   ├── tack_2/
│   │   └── tack_2.ino              # Main Arduino sketch (PD controller)
│   └── README.md
│
├── circuit/
│   ├── lin325.pdsprj               # Proteus 8 project file
│   ├── schematic_screenshot.png    # Schematic capture overview
│   └── README.md
│
├── mechanical/
│   ├── SolidWorks/                 # Full SOLIDWORKS 2022 assembly
│   │   ├── follow_line_robot.SLDASM
│   │   ├── Car_Chassis.SLDPRT
│   │   ├── Robot_Smart_Car_Kit_v2.SLDPRT
│   │   └── ...
│   ├── views/
│   │   ├── top_view.png
│   │   ├── bottom_view.png
│   │   ├── side_view_1.png
│   │   └── side_view_2.png
│   └── README.md
│
├── kinematics/
│   ├── Robot_Kinematics_Guide.pdf  # Full kinematic derivation (Forward, Inverse, Odometry, PID)
│   ├── tack_2.m                    # MATLAB simulation script
│   └── README.md
│
├── simulation/
│   ├── matlab_telemetry.png        # Speed, heading, error, deviation plots
│   └── README.md
│
├── docs/
│   └── IEEE_LFR_Final.docx         # IEEE-format project report
│
└── README.md                       # ← You are here
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

**File:** `embedded/tack_2/tack_2.ino`

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

All turns use `spinUntilLine()` — a sensor-guided spin (not timed) with a 1.5s safety timeout, which eliminates the need for manual delay tuning.

---

## Circuit Design (Proteus)

**File:** `circuit/lin325.pdsprj`  
**Tool:** Proteus 8 Professional

The schematic captures the full system:
- Arduino Uno as the central controller
- L298N motor driver connected to two DC motors
- 5× IR obstacle sensor modules (TCRT5000-compatible)
- Buzzer and LED for status indication
- Battery supply with flyback diodes for motor protection

The schematic was validated in Proteus simulation before hardware assembly.

---

## Mechanical Design (SolidWorks)

**Tool:** SOLIDWORKS Premium 2022 SP1.0  
**File:** `mechanical/SolidWorks/follow_line_robot.SLDASM`

### Assembly Breakdown

The robot body is a full parametric SolidWorks assembly including:

- **Car Chassis** — main body plate with motor mounts, sensor rail, and caster pocket
- **Robot Smart Car Kit v2** — drive wheel sub-assembly (2× TT motors + yellow spoke wheels)
- **Battery Box** — AA cell holder mounted on the upper deck
- **Blackboard v2.0** — Arduino Uno placeholder board
- **L298N module, HDR connectors, Tact DIP switch**
- **15× Pan-head screws** — full fastener constraints via Concentric + Coincident mates

All parts are fully mated with Concentric, Coincident, and Parallel constraints. A Motion Study was configured for animation.

### Views

| View | Description |
|------|-------------|
| Top | Battery, Arduino, and chassis top deck layout |
| Bottom | Motor placement, sensor bar, caster wheel |
| Side (front-left) | Drive wheel and motor assembly depth |
| Side (rear-right) | Caster, chassis, L298N bracket |

---

## Kinematics & Control Theory

**File:** `kinematics/Robot_Kinematics_Guide.pdf`

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

> **Note:** On microcontrollers (STM32/Arduino), use timer interrupts to keep Δt constant and avoid odometry drift and PID instability.

---

## MATLAB Simulation

**File:** `kinematics/tack_2.m`  
**Tool:** MATLAB R2024b

The simulation models the full robot on a rectangular closed track (Black Rose track) and produces a real-time telemetry dashboard with:

| Plot | Description |
|------|-------------|
| **Speed (m/s)** | Linear velocity over time — shows acceleration and turning slowdowns |
| **Heading (rad)** | Orientation in global frame — sharp spikes indicate 90° / U-turns |
| **Filtered Error** | Lateral deviation from line center after low-pass filtering |
| **Deviation (m)** | Absolute position error — peaks at corners, near-zero on straights |
| **Sensor Readings** | Bar chart of 5 sensor distances to line (L2, L1, C, R1, R2) |

The simulation includes **Restart** and **Reverse Direction** controls for interactive testing.

---

## Getting Started

### 1. Upload Arduino Code

```bash
# Open in Arduino IDE
File > Open > embedded/tack_2/tack_2.ino

# Select board: Arduino Uno
# Select correct COM port
# Upload
```

### 2. Run Proteus Simulation

```
Open circuit/lin325.pdsprj in Proteus 8
Press Play ▶ to simulate sensor response and motor behavior
```

### 3. Run MATLAB Simulation

```matlab
% In MATLAB R2024b or later
cd kinematics/
run('tack_2.m')
% Two figures open: track animation + telemetry dashboard
```

### 4. Open SolidWorks Assembly

```
Open mechanical/SolidWorks/follow_line_robot.SLDASM
Run Motion Study 1 for animation
```

---

## Results

- ✅ Robot successfully follows a black line on white surface including 90° corners and U-turns
- ✅ PD controller eliminates oscillation while maintaining responsive correction
- ✅ Sensor-guided turning (no delay-based timing) adapts automatically to any junction geometry
- ✅ MATLAB telemetry confirms heading stability and low lateral deviation on straight segments
- ✅ Proteus simulation validated wiring before hardware build
- ✅ SolidWorks assembly provides full mechanical reference for fabrication

---

## Team

**Black Rose Robotics Team**  
Mohamed Sayed Nageb
Ziad Asem Sadky
Ahmed Sayed Mahmoud, 
Ahmed Mohamed Ahmed Abdel Haleem
Fatma Khaled Mohamed Kamel
Elham Fadl Ahmed Ahmed
Rokaya Mansour Ibrahim Abdel Moneim
Farah Abdel Wahab Mohamed Abdel Wahab
Fatma Mahmoud Mohamed Sadek
Omnia El-Sayed Youssef zalama


---

## License

This project is submitted as an academic/competition project. All design files, code, and documentation are original work by the team.
