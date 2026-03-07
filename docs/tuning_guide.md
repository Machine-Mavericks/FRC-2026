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
*   `TARGET_TX_TOLERANCE_DEG`: This determines how perfectly the turrets need to be aligned before `isReadyToShoot()` allows the hopper to fire. It is currently set to `2.0` degrees. If the turrets have structural backlash and struggle to hit the `2.0` mark perfectly, you may need to increase this slightly so the robot doesn't get "stuck" refusing to fire.

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

---

## 4. Hopper & Uptake Tuning

The Hopper needs to forcefully push the game pieces into the Uptake, which speeds them into the flywheels. 

**Parameters to Tune (`RobotMap.Hopper` / `RobotMap.Uptake`):**
*   `HOPPER_SPEED` / `UPTAKE_SPEED`: Currently set to simple percent-outputs (`0.5` and `0.8`). If the balls are jamming, or conversely, if they are entering the shooter so violently they drop the flywheel speed too much before the second ball fires, adjust these feed rates.
*   **Stall Voltage & Current Limits (`Hopper.java` / `Uptake.java`):**
    *   Currently, the Hopper and Uptake do *not* have current limits cleanly exposed in `RobotMap`. 
    *   **Action Required:** In the subsystem constructors, use `config.CurrentLimits.SupplyCurrentLimit` on the TalonFX configurations. 
    *   *Why?* If a ball jams in the hopper or uptake, you want the motor to safely stall (current limit kicks in) rather than burning out the motor or snapping a belt. A good starting point is `30A` for a feeder mechanism.

---

## 5. Swerve Drive Calibration

**Parameters to Tune (`frc/robot/subsystems/SwerveDrive.java`):**
*   `MagnetOffset`: The absolute encoder offsets for the `LF`, `RF`, `LR`, and `RR` modules. 
    *   **How to tune:**
        1. Physically align all four wheels so they are perfectly straight (pointing forward). 
        2. Read the absolute position from Phoenix Tuner.
        3. Input those offsets into the `MagnetOffset` configurations so the wheels read `0.0` when straight.
*   `TRACK_WIDTH` / `TRACK_LENGTH`: Ensure these exactly match the physical wheel-center-to-wheel-center distances on your real robot, or autonomous rotation will be skewed.
