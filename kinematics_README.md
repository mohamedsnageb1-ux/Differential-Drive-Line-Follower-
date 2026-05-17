# Kinematics — Theory & Scripts

## Files

| File | Description |
|------|-------------|
| `Robot_Kinematics_Guide.pdf` | Full kinematic derivation — forward, inverse, odometry, PID |
| `Kinematics_final_version.rar` | Kinematics implementation scripts |

## Contents of the Guide

The PDF covers the complete mathematical model of the differential drive robot:

1. **Forward Kinematics** — maps wheel velocities (ωR, ωL) to body velocity (v, ω)
2. **Unicycle Model** — projects body velocities into the global (x, y, θ) frame
3. **Odometry Integration** — estimates robot pose over time using Δt steps
4. **Inverse Kinematics** — computes required wheel speeds from target (v, ω)
5. **PID Controller Design** — full derivation of the line-following controller

## How to Use

```bash
# Extract the kinematics scripts
unrar x Kinematics_final_version.rar

# Open the theory reference
# Read Robot_Kinematics_Guide.pdf
```

## Key Equations

**Forward Kinematics:**
```
v = (r/2)(ωR + ωL)
ω = (r/L)(ωR − ωL)
```

**Odometry (discrete integration):**
```
x(t) = x(t−Δt) + Δt · v · cos(θ)
y(t) = y(t−Δt) + Δt · v · sin(θ)
θ(t) = θ(t−Δt) + Δt · ω
```

**Inverse Kinematics:**
```
ωR = (v + ω·L/2) / r
ωL = (v − ω·L/2) / r
```
