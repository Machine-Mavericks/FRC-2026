# Hardware Smoke-Test Suite

**File:** `src/main/java/frc/robot/tests/HardwareTestSuite.java`  
**Triggered from:** `Robot.java` → `testPeriodic()`

---

## Purpose

The hardware smoke-test suite is a sequenced set of automated checks that verify
every major hardware subsystem is wired, communicating, and mechanically responding
as expected. It is intended for two scenarios:

- **Initial assembly** – confirm everything is operational before the first drive session.
- **Competition pit** – quick pre-match sanity check to catch anything that broke during
  transport, a crash, or a battery swap.

The entire sequence takes roughly **8–10 seconds** and requires no human interaction
beyond pressing a single button.

---

## How to Run

1. **Tether** the robot to a laptop running the FRC Driver Station.
2. Open **Shuffleboard** and navigate to the **"Hardware Tests"** tab (it is created
   automatically at robot startup).
3. Ensure the robot has **~15 cm (6 inches) of clear space directly in front of it**
   for the swerve drive test.
4. Set the Driver Station mode to **Test** and click **Enable**.
5. Press the **operator controller START button** (⊞ on Xbox) to begin the sequence.
6. Watch the Shuffleboard tab. Each row updates in real time as its test runs.

> **Safety note:** The robot will move a small amount during Test 2 (Swerve Drive).
> Keep all personnel clear. The motion is limited to 0.3 m/s for 0.5 s (≈ 15 cm).

---

## Shuffleboard Layout ("Hardware Tests" tab)

| Row | Label | Description |
|-----|-------|-------------|
| 0 | **Overall Status** | Summary: `ALL N TESTS PASSED ✓` or `N PASSED, M FAILED ✗` |
| 1 | **1. Gyro (Pigeon)** | Gyro test result with measured angles |
| 1 | **2. Swerve Drive** | Drive test result with peak wheel RPS |
| 2 | **3. Intake Motors** | Intake test result with peak RPS |
| 2 | **4. Intake Arm** | Arm test result with position error |
| 3 | **5. Uptake** | Uptake test result (or SKIPPED) |
| 3 | **6. Hopper Feed** | Hopper feed test result with RPS |
| 4 | **7. Shooter** | Shooter test result with peak RPS (or SKIPPED) |
| 5–7 | **HOW TO USE** | On-screen instructions |

Each cell shows one of:

| Status | Meaning |
|--------|---------|
| `---` | Not yet run |
| `RUNNING...` | Test is currently executing |
| `PASS ✓  <detail>` | Test passed; detail shows measured values |
| `FAIL ✗  <detail>` | Test failed; detail describes the symptom |
| `SKIPPED` | Test was skipped (see per-test notes below) |

The test can be **re-run** at any time by pressing START again.

---

## Individual Tests

### Test 1 — Gyro (Pigeon 2)

| Property | Value |
|----------|-------|
| Duration | Instant (no motion) |
| Robot moves? | No |
| Hardware required | Pigeon 2 on CAN ID `RobotMap.CANID.PIGEON` |

**What it does:** Reads yaw, pitch, and roll from the Pigeon 2.

**Pass criteria:**
- None of the three values is NaN (which would indicate a lost CAN connection).
- Neither pitch nor roll exceeds ±45° (robot is not badly tilted or tipped).

**Fail causes:** Pigeon not powered, wrong CAN ID, CAN bus fault, robot tipped on its side.

---

### Test 2 — Swerve Drive

| Property | Value |
|----------|-------|
| Duration | 0.5 s |
| Robot moves? | **YES — ~15 cm forward** |
| Hardware required | All 4 drive motors + 4 steer motors + Pigeon (for `FieldDrive`) |

**What it does:** Commands `RobotDrive(0.3, 0, 0, false)` for 0.5 s, then stops.
Reads the average absolute wheel velocity across all four drive motors while running.

**Pass criteria:** Average wheel velocity reaches ≥ 0.4 wheel RPS at any point during the pulse.
(At 0.3 m/s the expected value is ~1 RPS; the 0.4 threshold gives headroom for slow spin-up.)

**Fail causes:** Drive motor not powered, tripped breaker, CAN fault, wheels mechanically jammed.

---

### Test 3 — Intake Motors

| Property | Value |
|----------|-------|
| Duration | 0.75 s |
| Robot moves? | No |
| Hardware required | `INTAKE_MASTER` (CAN 23) + `INTAKE_FOLLOWER` (CAN 24) |

**What it does:** Calls `intake.intake()` for 0.75 s, reads master motor velocity throughout,
then stops.

**Pass criteria:** Peak absolute velocity ≥ 0.5 RPS.

**Fail causes:** Intake motor not powered, breaker tripped, CAN fault, roller jammed.

---

### Test 4 — Intake Arm

| Property | Value |
|----------|-------|
| Duration | Up to 2.5 s |
| Robot moves? | Arm moves |
| Hardware required | `INTAKE_ARM_RIGHT` (CAN 21) + `INTAKE_ARM_LEFT` (CAN 22) |

**What it does:** Commands the arm to `RobotMap.IntakeArm.STOWED_POSITION` using Motion Magic.
Waits up to 2.5 s for the encoder to converge, or exits early once within 0.5 rotations of target.

**Pass criteria:** Final encoder position is within 2.0 rotations of `STOWED_POSITION`.

**Fail causes:** Arm motor not powered, encoder not zeroed, soft limit misconfigured, mechanical bind.

> **Note:** Stowed position is always the safe end-stop so the arm will not slam into a hard stop.

---

### Test 5 — Uptake

| Property | Value |
|----------|-------|
| Duration | 0.75 s |
| Robot moves? | No |
| Hardware required | `UPTAKE_MASTER` (CAN 25) + `UPTAKE_FOLLOWER` (CAN 26) |
| Skipped when | `RobotMap.Features.ENABLE_LEFT_TURRET = false` |

**What it does:** Calls `uptake.feedShooter()` for 0.75 s, reads master motor velocity, then stops.

**Pass criteria:** Peak absolute velocity ≥ 0.5 RPS.

**Fail causes:** Motor not powered, breaker tripped, CAN fault.

**Skipped:** When `ENABLE_LEFT_TURRET` is `false` the hardware is replaced by a software stub
(`UptakeDisabled`) and the test is not meaningful.

---

### Test 6 — Hopper Feed

| Property | Value |
|----------|-------|
| Duration | 0.5 s wait (motor already running) |
| Robot moves? | No |
| Hardware required | `HOPPER_FEED` (CAN 27) |

**What it does:** The `HopperFeed` subsystem drives its motor automatically in its `periodic()`
method whenever the robot is enabled, so no explicit command is needed. This test simply waits
0.5 s for the motor to reach speed, then reads the velocity.

**Pass criteria:** Absolute motor velocity ≥ 0.5 RPS.

**Fail causes:** Motor not powered, breaker tripped, CAN fault.

---

### Test 7 — Shooter Flywheel

| Property | Value |
|----------|-------|
| Duration | Up to 2 s |
| Robot moves? | No |
| Hardware required | `SHOOTER` (CAN 20) |
| Skipped when | `RobotMap.Features.ENABLE_LEFT_TURRET = false` |

**What it does:** Commands the shooter to 20 RPS using its velocity closed-loop controller.
Tracks peak `shooter.velocity` (motor RPS) over 2 s, then stops.

**Pass criteria:** Peak velocity ≥ 10 RPS (50 % of target — provides margin for motors that
are slow to spin up).

**Fail causes:** Motor not powered, breaker tripped, CAN fault, belt/pulley broken.

**Skipped:** Same condition as Uptake — `ENABLE_LEFT_TURRET = false`.

---

## Re-running Tests

The test suite can be re-run as many times as needed within a single enable period by
pressing the operator START button again. Each run resets all status indicators to `---`
before starting.

---

## Adding New Tests

1. Add a diagnostic accessor method (e.g., `getVelocityRPS()`) to the new subsystem.
2. Write a private `testXxx()` method in `HardwareTestSuite.java` following the same
   `Commands.sequence(runOnce, runEnd+timeout, runOnce)` pattern.
3. Insert the new command into the `Commands.sequence(...)` call in `buildTestSequence()`.
4. Add a `GenericEntry` field and a matching `tab.add(...)` line in the constructor.

---

## Troubleshooting Common Failures

| Symptom | Likely cause |
|---------|-------------|
| All tests fail immediately | Robot not enabled / DS in wrong mode |
| Gyro shows NaN | CAN bus fault — check Pigeon power and CAN termination |
| Swerve FAIL, 0 RPS | All drive breakers tripped or main CAN issue |
| One motor subsystem FAIL, others PASS | That subsystem's breaker or CAN wire |
| Intake Arm FAIL (large position error) | Encoder not zeroed, arm mechanically bound, soft limit set wrong |
| Hopper Feed FAIL | Check CAN ID 27 is correct in `RobotMap` and breaker is set |
| Tests 5 & 7 both SKIPPED | Expected — `ENABLE_LEFT_TURRET = false` in `RobotMap.Features` |
