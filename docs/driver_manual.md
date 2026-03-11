# FRC 2026 Mavericks - Driver & Operator Manual

This document outlines the controls and responsibilities for the driver and operator of the FRC 2026 Mavericks robot.

## Controllers
* **Driver Controller:** Port 0
* **Operator Controller:** Port 1

---

## Driver Controls (Port 0)

The Driver is primarily responsible for moving the robot around the field. The robot uses a Swerve Drive system.

| Control | Action | Description |
| :--- | :--- | :--- |
| **Joysticks** | **Manual Drive** | Standard swerve drive layout (Left stick for translation, Right stick for rotation). |
| **Back Button** | **Reset Odometry** | Resets the robot's heading and odometry perfectly straight. Use this if the gyro drifts or you need to re-zero the orientation relative to the field. |

---

## Operator Controls (Port 1)

The Operator manages the robot's subsystems, including the intake, shooter, and turrets.

| Control | Action | Description |
| :--- | :--- | :--- |
| **Right Trigger** (Hold) | **Shoot Sequence** | Fires the cargo using the automated shooting sequence. Calculates required RPM automatically based on distance to the goal. |
| **Right Bumper** | **Toggle Intake** | First press: Deploys the intake arm and runs the intake motor.<br>Second press: Stops the intake motor and stows the arm. |
| **Left Bumper** (Hold) | **Manual Turret** | Overrides the Left and Right turrets' auto-tracking function, allowing for manual control of the turret. |
| **Y Button** | **Shooter Speed +15** | Increases the shooter speed offset by 15 RPM. |
| **X Button** | **Shooter Speed -15** | Decreases the shooter speed offset by 15 RPM. |
| **B Button** | **Shooter Speed +0.5** | Micro-adjusts the shooter speed offset up by +0.5 RPM. |
| **A Button** | **Shooter Speed -0.5** | Micro-adjusts the shooter speed offset down by -0.5 RPM. |

## Subsystem Behaviors

* **Turrets (Auto-Tracking):** By default, both the left and right turrets are set to automatically track the Hub targets using the Limelight vision system (`AutoTrackGoal`). Hold the Left Bumper on the Operator controller if you need to override this behavior.
* **Shooter Target Calculation:** The `HubTargetingSubsystem` automatically computes the required shooter RPM based on Limelight distance readings. The RPM trims (A, B, X, Y buttons) apply an offset to this calculated base RPM.
