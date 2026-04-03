# Intake Arm Tuning Guide

This is a specific, step-by-step guide for tuning the Intake Arm mechanism on the FRC 2026 robot. Proper tuning is critical because the arm uses two motors fighting gravity and physical hard stops.

---

## 1. Mechanism Overview
*   **Motors:** Two TalonFX motors (Right CAN ID 23, Left CAN ID 24).
*   **Control Style:** Independent Motion Magic Voltage (`m_mmReq.withPosition(...)`).
*   **Gravity Feedback:** Uses `GravityTypeValue.Arm_Cosine`. This requires $kG$ to be tuned perfectly for the maximum gravity torque point horizontally.
*   **Zero Position:** For the cosine gravity feedforward to work properly, the encoders **must interpret 0.0 rotations as perfectly horizontal**.

> [!WARNING]
> Because there are two motors operating independently to move the same shaft/mechanism, make sure the values are mirrored symmetrically and that both encoders are zeroed simultaneously to prevent mechanical binding.

---

## 2. Finding and Setting Physical Limits (BEFORE Tuning)

Since the total travel for the arm is less than 90 degrees, the default limits in `RobotMap.java` (0 to -90 degrees) are going to cause the arm to hit physical hard stops when moving. You must find your actual limits to protect the mechanism:

1. Keep the robot **Disabled**.
2. **Temporarily zero the encoder to Horizontal:** Using a bubble level or phone app, manually hold the arm perfectly horizontal. With the robot enabled in **Test Mode**, press the **Back Button** on the Operator Gamepad to zero the encoders (`0.0` rotations).
3. Disable the robot again. Open **Phoenix Tuner X** and connect to the robot.
4. Open the signals tab/device details for the Right Intake Arm Motor to read its **Position**.
5. Manually push the arm up until it gently rests against the **Upper Hard Stop**. Record this position in rotations (this will likely be greater than -0.25, e.g., -0.20 or -72 degrees).
6. Manually pull the arm down until it gently rests against the **Lower Hard Stop**. Record this position in rotations (e.g., -0.05).
7. Open `RobotMap.java` and update `FORWARD_SOFT_LIMIT` (bottom position) and `REVERSE_SOFT_LIMIT` (top position) to be slightly *inside* these physical hard stops (e.g., pad them by 0.01 rotations) so the software stops the motor before it hits the metal.
8. Update `DEPLOYED_POSITION` and `STOWED_POSITION` in `RobotMap.java` to fall safely within these new limits.
9. **Re-deploy your code.**

---

## 3. Setting Final Encoder Zero (Crucial, done after every reboot/deploy)

**Important:** Re-deploying your code in the previous step wiped out the temporarily zeroed position! The TalonFX encoder resets its zero point to whatever angle the arm is resting at whenever it boots or code is deployed.

If the arm is not perfectly horizontal when the robot boots up, gravity feedforward will fight the arm instead of helping it.

To safely zero the arm properly, use one of these methods:
*   **Preferred Method (Highest Accuracy):** Manually hold the arm so it is **perfectly horizontal**. Then, with the robot enabled in **Test Mode**, press the **Back Button** on the Operator Gamepad to zero the encoders (`0.0` rotations).
*   **Alternative Method (Using Stowed Position):** If you updated `STOWED_POSITION` in `RobotMap.java` in the previous step to precisely match your upper hard stop setting, let the arm rest against the top hard stop. With the robot enabled in **Test Mode**, press the **Start Button** to initialize the position to this stowed angle.

---

## 4. Tuning Gravity Feedforward ($kG$)

Because the arm uses `Arm_Cosine` gravity type, $kG$ represents the voltage required to hold the arm *perfectly horizontal* against gravity.

1. Ensure the encoders are correctly zeroed (see Step 2).
2. Physically hold the arm horizontally (or command it there if safe).
3. In **Phoenix Tuner X**, open the configuration for **both** the Left and Right Intake Arm motors.
4. Locate `Slot0.kG`. Start at `0.0`.
5. Slowly increase the $kG$ value simultaneously on both motors (you can use Phoenix Tuner's control tab to apply a neutral output and tune config live).
6. Find the *minimum* voltage value where the motors alone can hold the arm horizontal without it sagging downward.
7. Record this value cleanly in `frc/robot/subsystems/IntakeArm.java`.

---

## 5. Tuning Motion Magic (PID)

Once gravity is handled, tune the dynamic movement between the stowed and deployed positions.

**Test Mode Controls:**
*   **D-Pad Up:** Snaps the arm to `STOWED_POSITION`. *Only use this after you have updated `STOWED_POSITION` in `RobotMap.java` to match your actual top limit!* 
*   **D-Pad Down:** Snaps the arm to `DEPLOYED_POSITION`. *Only use this after updating `DEPLOYED_POSITION` to match your actual bottom limit!*

**Tuning the Gains (in `IntakeArm.java`):**
1. Test the arm movement using the D-Pad.
2. **Increase $kP$** (`Slot0.kP`) if the arm struggles to reach the target position or moves too sluggishly (current default is `4.0`).
3. **Increase $kD$** (`Slot0.kD`) if the arm violently overshoots the target and bounces when arriving (current default is `0.1`).
4. **Adjust Motion Magic Profile:**
   *   `MotionMagicCruiseVelocity` (default `0.5` rot/s): The maximum speed the arm travels. A value of `0.5` rot/s will ensure the arm takes at least half a second to move 90 degrees.
   *   `MotionMagicAcceleration` (default `1.0` rot/s²): How aggressively the arm ramps up to cruise velocity. Lower this if the arm jerks violently when starting or stopping.

---

## 6. Software and Hardware Limits

To protect the arm from tearing itself apart:

*   **Software Limits:** Ensure `FORWARD_SOFT_LIMIT` and `REVERSE_SOFT_LIMIT` in `RobotMap.IntakeArm` align tightly with your physical hard stops. The code automatically clamps commanded Motion Magic setpoints to stay within this range.
*   **Stator Current Limit:** `RobotMap.IntakeArm.STATOR_CURRENT_LIMIT` is currently set to `20.0` Amps. This cuts power to the motors if they stall against a hard stop or jam on a game piece. If the arm cannot lift a heavy game piece, you may need to increase this slightly (e.g. to `30.0` Amps or `40.0` Amps), but keep it as low as possible to protect the motors.
