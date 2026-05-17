# Embedded — Arduino Controller & Circuit

## Files

| File | Description |
|------|-------------|
| `line-follower.ino` | Main Arduino sketch — PD controller + turn detection |
| `schematic_screenshot.png` | Proteus 8 circuit schematic screenshot |

## How to Upload

1. Open `line-follower.ino` in Arduino IDE
2. Select **Board:** Arduino Uno
3. Select the correct **COM port**
4. Click **Upload**

## Key Parameters to Tune

Open `line-follower.ino` and adjust these at the top of the file:

```cpp
int   BASE_SPEED = 130;  // cruise speed (0–255)
int   MAX_SPEED  = 200;  // motor ceiling
int   TURN_SPEED = 150;  // speed while spinning on turns
float Kp = 35.0;         // proportional gain
float Kd = 20.0;         // derivative gain
```

**Tuning tips:**
- Start with `Kd = 0` and increase `Kp` until the robot oscillates, then back off slightly
- Add `Kd` gradually until oscillation is damped
- If turns overshoot, reduce `TURN_SPEED`
- If the robot loses the line on straights, reduce `BASE_SPEED`

## Circuit Overview

The schematic (`schematic_screenshot.png`) shows:
- Arduino Uno as central controller
- L298N motor driver connected to 2× DC motors
- 5× IR sensor modules (A0–A4)
- Buzzer on D13, Status LED on D7
- AA battery pack as power supply
