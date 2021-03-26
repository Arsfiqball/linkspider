# Definitions

### Indexing
| Index | Leg | Servo   | Axis | Frame | Angle |
| ----- | --- | ------- | ---- | ----- | ----- |
| 0     | L1  | SERVO_1 | X    | COXA  | TETA  |
| 1     | R1  | SERVO_2 | Y    | FEMUR | BETA  |
| 2     | L2  | SERVO_3 | Z    | TIBIA | ALPHA |
| 3     | R2  |         |      |       |       |
| 4     | L3  |         |      |       |       |
| 5     | R3  |         |      |       |       |

### Full Body
- **Leg Index**: Number or Alias (L1 = 0, R1 = 1, L2 = 2, R2 = 3, L3 = 4, R3 = 5) of each one leg.
- **Anchor Position**: Coordinate of leg anchor (x, y, z) which connects leg to body.
- **Tip Position**: Coordinate of leg tip (x, y, z) which touches the ground.
- **Leg Alignment**: Angle of leg on normalized position (θ = 0°) relative to X axis. For example, in this example below R1 has 45° alignment. 
- **Posture Rotation**: Pitch (rx), Roll (ry), Yaw (rz) of robot's body.
- **Normal Position**: Robot's standing posture when it's on standby mode.
- **FRS**: Front and Rear Span of Robot's Normal Position.
- **MS**: Middle Span of Robot's Normal Position.
- **L**: Length of Robot's Normal Position.

![Full Body](images/body.png)

### Single Leg
* **Servo Index**: Number or Alias of each Servo in one leg.
* **Servo Angle**: Angle of each servo (see image below for the reference).
* **H**: Height of Robot's Normal Position.
* **Normal PWM**: Each one of servo PWM value when the angle = 0°.
* **Ratio PWM**: Differential of how much angle shift per PWM (Δ angle / Δ pwm).

![Single Leg](images/leg.png)
