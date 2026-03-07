# HubTargetingSubsystem — How It Works

## Overview

`HubTargetingSubsystem` is a WPILib `SubsystemBase` that runs **every scheduler loop (~50 Hz)**.  
It reads the Limelight camera, decides whether the robot can "see" its alliance's HUB AprilTag, and publishes three key values for the rest of the robot to consume:

| Value | Method | Unit |
|---|---|---|
| Is a valid HUB tag in view? | `isHubTagVisible()` | boolean |
| Horizontal aim error | `getHorizontalAngleError()` | degrees (+ = target right) |
| Distance to HUB | `getDistanceToHub()` | meters |
| Shooter flywheel speed | `getShooterRPM()` | RPM |
| Are we aimed? | `isAimedAtHub()` | boolean |

---

## Data Flow (one scheduler loop)

```
Limelight
  ├── getHorizontalTargetOffsetAngle()  → tx  (horizontal aim error)
  ├── getVerticalTargetOffsetAngle()    → ty  (vertical angle to tag)
  └── getPrimAprilTagID()               → tag ID

TargetCalculations.isOurHubTag(tagID, isRedAlliance)
  └── checks tag ID against RobotMap.Vision.BLUE/RED_HUB_TAG_IDS
        ↓ valid tag?
ShooterCalculations.calculateDistanceFromTy(ty)
  └── distance = (tagHeight − cameraHeight) / tan(mountAngle + ty)
        ↓
ShooterCalculations.getShooterRPM(distance)
  └── linear interpolation through DISTANCE_RPM_TABLE
        ↓
Shuffleboard  +  public getters  →  AutoTrackGoal / ShooterSubsystem
```

---

## Key Classes

### `HubTargetingSubsystem` (`subsystems/`)
The central state machine. Runs in `periodic()`, publishes all computed state via getters.

### `ShooterCalculations` (`utils/`)
Two jobs:
1. **Distance from camera** — `calculateDistanceFromTy(ty)` uses the triangle formed by the known camera-mount height, camera tilt angle, and tag height above carpet.
2. **RPM from distance** — `getShooterRPM(distance)` linearly interpolates a lookup table ranging from 1 m (2000 RPM) to 7.5 m (5200 RPM).

### `TargetCalculations` (`utils/`)
Two modes:
- **Direct Limelight mode** — `isOnTarget(tx, tolerance)` checks if `|tx| ≤ 2°` (the "aimed at hub" check).
- **Field-coordinate mode** — `getTargetAngleForTurret(robotPose, ...)` computes where the turret should point based on odometry + known HUB position on the field. Used as a fallback when no tag is visible.

---

## "Aimed at Hub" Logic

```java
isAimedAtHub() == isHubTagVisible() && |tx| <= TARGET_TX_TOLERANCE_DEG (2.0°)
```

Gate your shoot command on `isAimedAtHub() && turret.atSetpoint()`.

---

## Shuffleboard Tab: `"Hub Targeting"`

| Column | Widgets |
|---|---|
| Status | Hub Tag Visible, Tag ID, Aimed at Hub |
| Camera Angles | tx (horiz error °), ty (vert angle °) |
| Calculated | Distance to Hub (m), Shooter RPM |

---

## ⚠️ Tuning Checklist (before competition)

1. **`RobotMap.Vision.LIMELIGHT_MOUNT_HEIGHT_M`** — measure camera center height above carpet
2. **`RobotMap.Vision.LIMELIGHT_MOUNT_ANGLE_DEG`** — measure camera upward tilt angle
3. **`RobotMap.Vision.BLUE/RED_HUB_TAG_IDS`** — verify against the game manual's field diagram
4. **`ShooterCalculations.DISTANCE_RPM_TABLE`** — tune empirically; current values are placeholders
5. **`TARGET_TX_TOLERANCE_DEG`** (in `HubTargetingSubsystem`) — loosen (e.g. 3°) if turret backlash causes hunting; tighten for more accuracy

---

## Field-Coordinate Fallback

When no tag is visible, `AutoTrackGoal` can call `TargetCalculations.getTargetAngleForTurret(robotPose, ...)` using odometry to keep the turret roughly pointed at the HUB. This is less accurate than the Limelight path but keeps the robot "warm" between tag sightings.
