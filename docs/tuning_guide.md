# FRC 2026 Robot Tuning Guide

This document outlines the key configurable parameters that need to be tuned on the physical robot to ensure all subsystems perform accurately and reliably. All of these values are located in `src/main/java/frc/robot/RobotMap.java` unless otherwise specified.

---

## 1. Vision & Aiming (Limelights)

The robot now uses two separate Limelights: one for the Drive (Odometry/Pose Estimation) and one for the Shooter (Hub Targeting). The `HubTargetingSubsystem` entirely relies on accurate physical constants to calculate the distance to the HUB. If these are wrong, your shooter RPM lookup table will be useless.

**Parameters to Tune (`RobotMap.Vision`):**
*   `SHOOTER_LIMELIGHT_MOUNT_HEIGHT_M`: Measure the straight-line distance from the floor carpet to the exact center of the Shooter Limelight lens.
*   `SHOOTER_LIMELIGHT_MOUNT_ANGLE_DEG`: Use a digital angle gauge/level to measure the upward tilt of the Shooter Limelight relative to a perfectly flat floor. 
    *   *Tip:* If your distance calculations consistently read too short or too far across the board, this angle is likely off by a degree or two.

> [!NOTE]
> The position of the Drive Limelight on the robot has not yet been determined. Once it is physically mounted, you will need to add new mount height and angle constants for it in `RobotMap.Vision`, similar to the Shooter Limelight, and update `Odometry.java` (or any other pathing logic) to use those new measurements if required for accurate field pose projection.

**3D Pose Configuration (Java Code):**
In addition to the simple straight-line trigonometric distance values configured above, the Limelights require exact 3D positioning (X, Y, Z translation and Roll, Pitch, Yaw rotation) relative to the physical center of the robot. This allows the Limelights to accurately project the robot's pose on the field (`botpose`) for Odometry and MegaTag localization. 

This 3D transform is pushed to the cameras **from the RoboRIO** over NetworkTables (`camerapose_robotspace_set`) on startup. It overrides any geometry configured in the Limelight Web UI.
*   Locate the `SHOOTER_LIMELIGHT_3D_POSE` and `DRIVE_LIMELIGHT_3D_POSE` arrays in `RobotMap.Vision`.
*   Measure the exact Forward, Side, and Up (Height) offsets in meters from the center of the robot to each camera lens.
*   Note the Roll, Pitch, and Yaw in degrees. (*Note: the `SHOOTER_` array already uses the generic Height and Pitch constants from above.*)
*   Enter these values into the arrays in `[Forward, Side, Up, Roll, Pitch, Yaw]` order.

**Tolerance (`HubTargetingSubsystem.java`):**
*   `TARGET_TX_TOLERANCE_DEG`: This determines how perfectly the turrets need to be aligned before `isReadyToShoot()` allows the intake arm to fire. It is currently set to `2.0` degrees. If the turrets have structural backlash and struggle to hit the `2.0` mark perfectly, you may need to increase this slightly so the robot doesn't get "stuck" refusing to fire.

---

## 2. Shooter Flywheel Tuning

The shooter speed maps calculated distance to the goal directly to a desired flywheel RPM.

**Parameters to Tune (`frc/robot/utils/ShooterCalculations.java`):**
*   `DISTANCE_RPM_TABLE`: This is currently populated with placeholder data. 
    *   **How to tune:** 
        1. Place the robot exactly 1.0 meters from the HUB.
        2. Manually adjust the RPM until the shots consistently score in the center.
        3. Record that RPM in the table next to `1.0`.
        4. Move back 0.5 meters and repeat until you hit your maximum shooting range.
    *   *Tip:* The code automatically uses linear interpolation to seamlessly guess the perfect RPM for distances between your recorded points (e.g., 1.25 meters).

---

## 3. Turret PID & Limits

The independent left and right turrets need precise PID tuning to aim quickly without oscillating.

**Parameters to Tune (`RobotMap.Turret`):**
*   `MAX_ROTATION_DEGREES` / `MIN_ROTATION_DEGREES`: Update these based on the physical hard stops of your mechanism to prevent the turrets from tearing out their own wires.
*   `LEFT_TURRET_X_OFFSET`, `LEFT_TURRET_Y_OFFSET`, etc: Used for Odometry fallback targeting. Measure the physical distance from the exact center of the robot to the center of rotation for each turret.
*   `kP, kI, kD`: Standard PID gains. The current values are placeholders. Tune these using Phoenix Tuner first to ensure the turrets snap to their targets without excessive jitter.

### 3a. Tuning Procedure (using Test Mode)

We have built dedicated tuning controls into **Test Mode** (in `Robot.java`) to make this safe and easy.

**Step 1: Set Position / Zero Encoders**
You MUST do this first so the robot knows where straight ahead is.
1. Enable the robot in **Test Mode**.
2. Manually rotate the turrets so they are pointing perfectly straight ahead.
3. Press the **Left Bumper** on the Operator controller to zero the encoders (0 deg).

**Step 2: Tune PID Gains**
1. While still in Test Mode, use the **Operator A Button and D-Pad** to snap between positions:
   * **A Button:** Move to `0.0` (Center)
   * **D-Pad Left:** Move to `MIN_ROTATION_DEGREES` (-90 deg)
   * **D-Pad Right:** Move to `MAX_ROTATION_DEGREES` (90 deg)
2. Adjust `kP` (currently `0.05`), `kI` (currently `0.0`), and `kD` (currently `0.001`) in `RobotMap.Turret` until the turrets snap cleanly without violent overshoot or sluggishness.

---

## 4. IntakeArm & Uptake Tuning

The IntakeArm has a finite range of travel and uses Motion Magic (TalonFX on-controller closed-loop) to move to commanded positions. Software soft limits, stator current limiting, and optional hardware limit switches protect the mechanism.

### 4a. Zeroing and Soft Limits (Horizontal = 0)

**Important Concept:** To properly calculate gravity feedforward, the arm's angle relative to gravity must be known. In this codebase, **0 rotations = PERFECTLY HORIZONTAL**.

**Parameters to Tune (`RobotMap.IntakeArm`):**
*   `FORWARD_SOFT_LIMIT` *(currently `90.0 / 360.0` rotations)* — motor rotations at the fully retracted (home/stowed) straight-up position.
*   `REVERSE_SOFT_LIMIT` *(currently `0.0 / 360.0` rotations)* — motor rotations corresponding to the arm fully deployed (down). 

### 4b. Tuning Procedure (using Test Mode)

We have built dedicated tuning controls into **Test Mode** (in `Robot.java`) to make this safe and easy. 

**Step 1: Set Position / Zero Encoders**
You MUST do this first so the robot knows where horizontal is.
1. Enable the robot in **Test Mode**.
2. **OPTION A (Easy Way):** If you let the arm boot up resting naturally against its straight-up (stowed) hard stop, simply press the **Start** button on the Operator controller. This forces the encoder to the `STOWED_POSITION` (+90 deg).
3. **OPTION B (Manual Way):** If you physically moved the arm so it is perfectly horizontal before enabling, press the **Back** button on the Operator controller to zero the encoders (0 deg).

**Step 2: Tune Gravity Feedforward ($kG$)**
1. Ensure the arm is free to move.
2. In Phoenix Tuner X, find `Slot0` for the IntakeArm motors.
3. Slowly increase `kG` (starting from 0.0).
4. Find the *minimum* voltage value where `kG` alone can hold the arm horizontal against gravity without dropping. Record this value in `IntakeArm.java`.

**Step 3: Tune Motion Magic PID**
1. While still in Test Mode, use the **Operator D-Pad** to snap between positions:
   * **D-Pad Up:** Move to `STOWED_POSITION`
   * **D-Pad Down:** Move to `DEPLOYED_POSITION`
2. Adjust `kP` (currently `12.0`) and `kD` (currently `0.1`) in `IntakeArm.java` until the arm snaps cleanly without violent overshoot or sluggishness.

| Constant | Current Value | Description |
|---|---|---|
| `Slot0.kG` | `0.0` | Gravity feedforward — Voltage to hold arm horizontal (**MUST TUNE**) |
| `Slot0.kP` | `12.0` | Proportional gain — increase if arm is sluggish, decrease if it oscillates |
| `Slot0.kD` | `0.1` | Derivative gain — increase to dampen overshoot at end of motion |
| `MotionMagicCruiseVelocity` | `5` rot/s | Max cruise velocity — reduce to slow arm travel |
| `MotionMagicAcceleration` | `10` rot/s² | Ramp-up/down rate — reduce if arm jerks on start/stop |

### 4c. Stator Current Limit

**Parameter (`RobotMap.IntakeArm`):**
*   `STATOR_CURRENT_LIMIT` *(currently `40.0` A)* — protects the motor from stall damage when the arm reaches a hard stop. Lower values provide more protection but reduce peak torque. `30–50 A` is a reasonable range for an arm mechanism.

### 4e. Uptake Feed Rate

**Parameter (`RobotMap.Uptake`):**
*   `UPTAKE_SPEED` *(currently `0.8`)* — simple percent output. If balls are jamming, reduce speed. If flywheel speed drops too much before the second shot, reduce speed to smooth feeding.

---

## 5. Swerve Drive Calibration

**Parameters to Tune (`frc/robot/subsystems/SwerveDrive.java`):**
*   `MagnetOffset`: The absolute encoder offsets for the `LF`, `RF`, `LR`, and `RR` modules. 
    *   **How to tune:**
        1. Physically align all four wheels so they are perfectly straight (pointing forward). 
        2. Read the absolute position from Phoenix Tuner.
        3. Input those offsets into the `MagnetOffset` configurations so the wheels read `0.0` when straight.
*   `TRACK_WIDTH` / `TRACK_LENGTH`: Ensure these exactly match the physical wheel-center-to-wheel-center distances on your real robot, or autonomous rotation will be skewed.

---

## 6. HopperFeed Tuning

The HopperFeed feeds game pieces toward the shooter. It uses a TalonFX and can be tuned via Shuffleboard.

**Parameters to Tune:**
*   `Feed Speed` (Shuffleboard Tab: "HopperFeed"): The default speed is `RobotMap.HopperFeed.DEFAULT_SPEED`. Adjust this from the dashboard if balls are jamming or not reaching the shooter fast enough.
*   **Open Loop Ramps** (`HopperFeed.java`): The `VoltageOpenLoopRampPeriod` and `DutyCycleOpenLoopRampPeriod` are both set to 10 seconds. Reduce this if the hopper takes too long to reach full feeding speed.
