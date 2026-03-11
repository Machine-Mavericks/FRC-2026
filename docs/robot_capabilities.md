# 🤖 FRC 2026 Mavericks: Robot Capabilities & Command Overview

Welcome to the **FRC 2026 Mavericks** robot overview! Our machine this year is packed with advanced automation, dual-vision systems, and a highly capable dual-turret scoring mechanism. It's built to dominate the field with speed, precision, and smart software.

Here is a full breakdown of what our robot brings to the competition, along with a quick-reference guide for the Driver and Operator.

---

## 🌟 Core Systems & Capabilities

### 🏎️ Swerve Drive & Odometry
* **Omnidirectional Agility:** Full swerve drive allows us to translate in any direction while independently controlling the robot's rotation.
* **Field-Centric Driving:** The driver commands field-relative motion using the onboard **Pigeon Gyro**, meaning "push forward" always moves the robot downfield, regardless of which way the front bumper faces.
* **Odometry Tracking:** The robot constantly tracks its X/Y field position and heading, making advanced autonomous routines and path-following possible.

### 👀 Dual-Limelight Vision System
We run *two* distinct Limelight cameras to maximize data throughput without compromising tracking speed:
1. **Drive Limelight (`limelightDrive`):** Dedicated to field localization and odometry updates.
2. **Shooter Limelight (`limelightShooter`):** Dedicated exclusively to tracking the Hub (Blue and Red targets). It feeds lightning-fast data directly into the Hub Targeting Subsystem.

### 🎯 Intelligent Turrets & Targeting
* **Dual Action Turrets:** We feature both **Left and Right Turrets** allowing for multi-directional scoring and tracking.
* **Auto-Tracking (`AutoTrackGoal`):** Both turrets continually lock onto the Hub automatically. Overrides are available, but the system does the heavy lifting for you!
* **Automated Ballistics (`HubTargetingSubsystem`):** You don't need to guess the speed. The robot reads the Limelight distance, calculates the exact shooter RPM required to hit the target, and spins the motors to match.

### ⚙️ Intake, Hopper, and Shooter
* **Deployable Intake Arm:** A pneumatically/motor-actuated arm drops down seamlessly to grab cargo off the floor and stows itself safely inside the frame perimeter when not in use.
* **Integrated Feeding (Hopper & Uptake):** Cargo is smoothly fed from the intake, through the hopper, and staged in the uptake, ready for rapid-fire.
* **Automated Shoot Sequence:** A single button hold coordinates the uptake, hopper, and shooter wheel to fire at exactly the right moment.

---

## 🎮 Operator & Driver Controls

Controlling the Mavericks robot is a team effort. The controls are split securely between the Driver (moving the robot) and the Operator (managing scoring and subsystems).

### 🕹️ Driver Controls (Controller Port 0)

| Input | Action | Description |
| :--- | :--- | :--- |
| **Left Joystick** | **Translate (X/Y)** | Move the robot forward, backward, left, or right relative to the field. |
| **Right Joystick (X)** | **Rotate** | Spin the robot chassis clockwise or counter-clockwise. |
| **Back Button** | **Zero Odometry** | If the gyro drifts, hit this to reset your heading to exactly `0` degrees forward. |

### 🎛️ Operator Controls (Controller Port 1)

| Input | Action | Description |
| :--- | :--- | :--- |
| **Right Trigger** (Hold) | **🔥 FIRE (Shoot Sequence)** | The ultimate button. Holding this runs the Uptake and Hopper, firing the cargo once the Shooter reaches the auto-calculated RPM. |
| **Right Bumper** | **📥 Toggle Intake** | Press once to drop the arm and suck in cargo. Press again to turn off and retract. |
| **Left Bumper** (Hold) | **✋ Manual Turret** | Overrides Limelight Auto-Tracking. While holding, use **Left/Right Stick Y-axes** to independently control the Left/Right turrets. |

#### 🔧 Shooter Trim Controls
Sometimes the match dynamics change (battery sag, ball wear). The Operator can "trim" the auto-calculated target RPM up or down on the fly!

| Button | Action |
| :--- | :--- |
| **Y Button** | +15 RPM (Macro Up) |
| **X Button** | -15 RPM (Macro Down) |
| **B Button** | +0.5 RPM (Micro Up) |
| **A Button** | -0.5 RPM (Micro Down) |

---
*Stay sharp, communicate clearly, and let the software handle the math. Let's go Mavericks!* 🚀
