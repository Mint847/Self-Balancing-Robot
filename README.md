# Self-Balancing Robot (ESP32)

A two-wheeled self-balancing robot built on an ESP32 with N20 gear motors and encoders.
You can tune it live from your phone over WiFi while it balances.

## What it does
- Balances upright using an MPU6050 and a PID controller running at 200 Hz.
- Uses the wheel encoders to stop it drifting away: if it starts rolling, it leans back against the motion.
- Learns the balance angle by itself on slopes (auto-trim).
- Hosts its own WiFi network with a tuner page: change the PID values, see live pitch/speed, calibrate the gyro and test the encoders. Settings are saved in flash.

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
2. Open `SelfBalance/SelfBalance.ino`.
3. **Change `AP_PASS`** (the WiFi password, at least 8 characters).
4. Upload to the ESP32.
5. Connect your phone to the `SelfBalance-Tuner` WiFi and open `192.168.4.1`.

## First run
1. Lift the robot so the wheels spin freely and press **Test Encoders**. It detects each wheel's direction (and any dead encoder) and saves the result.
2. Place the robot still and press **Calibrate Gyro**.
3. Set your balance angle (**Setpoint**) and press **Apply Values**.
4. Stand it up and tune.

## Tuning guide
| Symptom | Try |
|---|---|
| Fast shaking | lower Kp or raise Kd |
| Slowly rolls away and doesn't come back | raise Kv (anti-drift) |
| Slow back-and-forth swinging | lower Kv or Kx |
| Small corrections do nothing | raise Deadband |
| More Kd makes it worse | set `GYRO_SIGN` to `-1.0f` in the code |
| Drifts on a slope | raise Auto-trim |

Typical starting values: Kp 45, Ki 1-3, Kd 0.5-1, Kv 0.5, Kx 0.04, Auto-trim 0.3.

## How it works (short version)
- **Inner loop:** PID on the pitch angle, with the gyro rate used for the derivative term.
- **Outer loop:** wheel speed and position from the encoders shift the target lean angle.
- **Auto-trim:** a persistent lean request is slowly folded into the balance angle.
- The web server runs on the other ESP32 core so it can't slow down the balance loop.

## Roadmap
- Gamepad control from a PC
- OLED display and speaker (companion robot)

## License
See [LICENSE](LICENSE).
