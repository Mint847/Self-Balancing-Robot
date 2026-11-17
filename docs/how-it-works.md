# How it works

## Overview
```
MPU6050 --> pitch + gyro rate --> [inner loop: angle PID] --> motor PWM
wheel encoders --> speed/position --> [outer loop] --> lean offset --^
drive buttons --> target speed ---------^
```

## 1. Sensing
- The MPU6050 is read every 5 ms (200 Hz) over I2C at 400 kHz.
- The `MPU6050_light` library fuses accelerometer and gyro with a complementary filter (98% gyro, 2% accelerometer). That gives a smooth `pitch` angle that doesn't drift.
- At boot the angle estimate is warmed up for about 1.5 s with a lower gyro weight, so it starts from the real tilt and not 0 degrees. Keep the robot still and upright while it boots.
- Gyro bias offsets are measured once (Calibrate Gyro) and stored in flash. The accelerometer is never calibrated, because the balance angle is set by hand.

## 2. Inner loop: balance (200 Hz)
```
err = pitch - target_angle
u   = Kp*err + Ki*integral(err) + Kd*gyro_rate
```
- `Kd` multiplies the measured gyro rate directly, not a difference of angles, so the damping is smooth and not noisy.
- The integral is clamped (+-100 PWM) to prevent windup.
- **Deadband compensation:** small motors need a minimum PWM to start turning. A non-zero `Deadband` adds that offset so small corrections still move the wheels.
- The output (-255..255) goes to both motors.

## 3. Outer loop: anti-drift (25 Hz)
Tilt alone can't tell standing still from slowly rolling, so the encoders close the loop:
```
speed        = average wheel speed (ticks/s), filtered
target_lean  = -(Kv * (speed - speed_target) + Kx * position) / 100
target_angle = setpoint + trim + target_lean
```
- Rolling forward lowers the target angle, so the robot leans back and brakes. Rolling backward does the opposite.
- `Kv` reacts to speed; `Kx` pulls it back toward where it was.
- The lean is limited to +-5 degrees, which also caps how hard it can accelerate.
- Only working encoders are used. Each wheel has a saved direction sign, because a backwards-counting encoder would flip the sign of the feedback and make drift worse.

## 4. Auto-trim (slopes)
If the outer loop keeps asking for the same lean, the real balance angle has moved (slope, load, battery). A slow integrator folds that lean into `trim`, which is added to the setpoint, up to +-15 degrees. It only learns while the robot is in control, and it is paused while driving.

## 5. Driving
- The Drive pad sends `/drive?f=<-1..1>&t=<-1..1>` about every 100 ms while a button is held.
- Forward/back sets `speed_target = f * Drive speed`, ramped at 400 ticks/s per second so starts and stops are smooth.
- Turning adds `t * Turn power` to one wheel and subtracts it from the other.
- Failsafe: with no command for 300 ms, both commands go to 0. While driving, the position term and trim learning are paused.

## 6. Safety
- Tilt more than 35 degrees from the balance angle: motors off, state FALLEN. It resumes when stood up within 8 degrees.
- Stop button: motors off and all control state reset.

## 7. Two cores
Balancing runs in `loop()` on core 1. The web server runs in its own task on core 0, so serving the page can never delay a control step. The web side never touches I2C or the motors; it only sets request flags (gyro calibration, encoder test) that the control loop executes.

## 8. Web endpoints
| Endpoint | Purpose |
|---|---|
| `/` | the control page |
| `/config` | all tunable values as JSON |
| `/state` | live telemetry as JSON |
| `/set?kp=...&ki=...` | change and save values |
| `/drive?f=..&t=..` | drive command |
| `/stop?state=0` or `1` | stop / resume motors |
| `/calgyro` | calibrate gyro |
| `/enctest` | encoder direction test |

## 9. Saved settings
Stored in ESP32 flash (Preferences namespace `balance`): PID values, setpoint, deadband, Kv, Kx, Auto-trim, drive speed, turn power, encoder signs and OK flags, and gyro offsets.

## Parameters
| Name | Meaning |
|---|---|
| Kp / Ki / Kd | angle PID gains |
| Setpoint | balance angle by hand (degrees) |
| Deadband | minimum motor PWM |
| Kv | lean per 100 ticks/s of speed error |
| Kx | lean per 100 ticks of position error |
| Auto-trim | how fast the balance angle adapts to slopes |
| Drive speed | wheel speed at full forward (ticks/s) |
| Turn power | PWM difference between wheels when turning |
