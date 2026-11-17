# Self-Balancing Robot (ESP32)

A two-wheeled self-balancing robot on an ESP32 with N20 gear motors and encoders.
It balances on its own, recovers from pushes, and can be driven and tuned from your phone over WiFi.

**Demo video:** [media/balancing-demo.mov](media/balancing-demo.mov)

## Features
- 200 Hz balance loop (PID on pitch, gyro rate for damping)
- Encoder-based anti-drift loop: if it starts rolling away, it leans back against the motion
- Auto-trim: learns the balance angle on slopes by itself
- Phone-friendly web page served by the ESP32 itself: live telemetry, tuning sliders, 4-way drive pad
- All settings saved in flash, so they survive a power cycle
- Self-test tools: encoder direction test (also finds a dead encoder) and gyro calibration

## Hardware
| Part | Notes |
|---|---|
| ESP32 DOIT DevKit V1 | main controller |
| MPU6050 | IMU (accelerometer + gyro) |
| 2x N20 gear motors with encoders | drive wheels + wheel speed feedback |
| Dual H-bridge motor driver | PWM + two direction pins per motor |
| 7.4V 2S LiPo | battery |

**Power (recommended):** battery to the motor driver, a regulated 5 V supply to the ESP32 VIN, the MPU6050 on 3.3 V, and a common ground everywhere. Check your encoders' voltage: ESP32 inputs are 3.3 V only.

## Wiring
| Function | GPIO |
|---|---|
| Left motor PWM / IN1 / IN2 | 19 / 18 / 5 |
| Right motor PWM / IN1 / IN2 | 25 / 26 / 27 |
| Left encoder A / B | 32 / 33 |
| Right encoder A / B | 13 / 14 |
| MPU6050 SDA / SCL | 21 / 22 |
| Status LED | 4 |

## Setup
1. Install the ESP32 board package and the **MPU6050_light** library in the Arduino IDE.
2. Open `SelfBalance/SelfBalance.ino` and **change `AP_PASS`** (the WiFi password, at least 8 characters).
3. Upload. (Don't enable "Erase all flash before upload", or your saved settings are lost.)
4. Connect your phone to the `SelfBalance-Tuner` WiFi and open `192.168.4.1`.

## First run
1. Lift the robot so the wheels spin freely and press **Test Encoders** (System section). It detects each wheel's direction and any dead encoder, and saves the result.
2. Hold the robot still and press **Calibrate Gyro**.
3. Set the **Setpoint** (the angle where it balances by hand) and press **Apply & Save**.
4. Stand it up and tune.

## Driving
Hold the arrow buttons on the Drive pad (or arrow keys / WASD on a PC). Release to stop.
If forward goes backward, set `DRIVE_SIGN` to `-1.0f`; if left/right are swapped, set `TURN_SIGN` to `-1.0f`.
If the page loses connection while you hold a button, the robot stops driving within 0.3 s.

## Tuning guide
| Symptom | Try |
|---|---|
| Fast shaking | lower Kp or raise Kd |
| Slowly rolls away and doesn't come back | raise Kv |
| Slow back-and-forth swinging | lower Kv or Kx |
| Small corrections do nothing | raise Deadband |
| More Kd makes it worse | set `GYRO_SIGN` to `-1.0f` in the code |
| Drifts on a slope | raise Auto-trim |

Typical values: Kp 45, Ki 1-3, Kd 0.5-1, Kv 0.5, Kx 0.04, Auto-trim 0.3.

## How it works
Short version: a fast inner loop keeps the robot upright, and a slower outer loop uses the wheel encoders to keep it from rolling away.
The full explanation, with the control equations and all web endpoints, is in [docs/how-it-works.md](docs/how-it-works.md).

## License
See [LICENSE](LICENSE).
