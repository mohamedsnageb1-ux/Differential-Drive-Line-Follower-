# Mechanical — SolidWorks Design & MATLAB Simulation

## Files

| File | Description |
|------|-------------|
| `solidworks_top_view.png` | Top view — battery, Arduino, and chassis layout |
| `solidworks_bottom_view.png` | Bottom view — motors, sensor bar, caster wheel |
| `solidworks_side_view_1.png` | Side view — drive wheel and motor assembly |
| `solidworks_side_view_2.png` | Side view — caster, chassis, L298N bracket |
| `tack_2.m` | MATLAB simulation script |
| `matlab_telemetry.png` | Telemetry dashboard screenshot |

## SolidWorks Assembly

**Tool:** SOLIDWORKS Premium 2022 SP1.0

The full assembly includes:
- Car chassis (main body plate)
- 2× TT DC motors with yellow spoke wheels
- 1× Ball caster wheel (front)
- Battery box (AA cells)
- Arduino Uno board (Blackboard v2.0)
- L298N motor driver module
- 15× Pan-head screws with full mate constraints

All parts are constrained with Concentric, Coincident, and Parallel mates. A Motion Study is included for animation.

## MATLAB Simulation

**Tool:** MATLAB R2024b

### How to Run

```matlab
cd mechanical/
run('tack_2.m')
```

Two windows will open:
- **Figure 1** — Black Rose track animation with robot position in real time
- **Figure 2** — Telemetry & Control dashboard

### Telemetry Dashboard

| Plot | What to Look For |
|------|-----------------|
| Speed (m/s) | Should be steady on straights, dips on turns |
| Heading (rad) | Sharp spikes = 90° turn or U-turn detected |
| Filtered Error | Should oscillate near 0 on straights |
| Deviation (m) | Peaks at corners — lower is better |
| Sensor Readings | Bar chart of L2, L1, C, R1, R2 distances to line |

Use the **Restart** button to reset the simulation and **Reverse Direction** to run the track in the opposite direction.
