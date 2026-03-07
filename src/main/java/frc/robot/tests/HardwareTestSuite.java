package frc.robot.tests;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

/**
 * Hardware smoke-test suite for FRC 206.
 *
 * <p>Intended for use immediately after assembly and at competition to verify
 * all hardware is operational before a match.
 *
 * <h3>How to use</h3>
 * <ol>
 *   <li>Tether the robot to a laptop running the Driver Station.</li>
 *   <li>Make sure the robot has <strong>clearance to move</strong> ~10 cm in
 *       the forward direction (for the swerve test).</li>
 *   <li>Set the Driver Station mode to <strong>Test</strong> and enable.</li>
 *   <li>Press the <strong>operator START button</strong> to begin the sequence.</li>
 *   <li>Watch the <em>Hardware Tests</em> Shuffleboard tab for results.</li>
 * </ol>
 *
 * <h3>Tests performed</h3>
 * <ol>
 *   <li><b>Gyro (Pigeon)</b> – reads yaw/pitch/roll; checks for NaN and sane orientation.</li>
 *   <li><b>Swerve Drive</b> – brief 0.3 m/s forward pulse; confirms wheel RPS feedback.</li>
 *   <li><b>Intake Motors</b> – runs intake at normal speed for 0.75 s; checks RPS.</li>
 *   <li><b>Intake Arm</b> – commands stow position; checks encoder tracks the command.</li>
 *   <li><b>Uptake</b> – runs for 0.75 s; checks RPS (SKIPPED if hardware disabled).</li>
 *   <li><b>Hopper Feed</b> – runs automatically when enabled; reads velocity after 0.5 s.</li>
 *   <li><b>Shooter</b> – spins to 20 RPS for 2 s; checks it reaches ≥10 RPS (SKIPPED if disabled).</li>
 * </ol>
 */
public class HardwareTestSuite {

    // ── Status string constants ─────────────────────────────────────────────
    private static final String PENDING = "---";
    private static final String RUNNING = "RUNNING...";
    private static final String PASS    = "PASS \u2713";   // ✓
    private static final String FAIL    = "FAIL \u2717";   // ✗
    private static final String SKIPPED = "SKIPPED";

    // ── Pass/fail counters (reset each run) ──────────────────────────────────
    private int passCount;
    private int failCount;

    // ── Shuffleboard entries ─────────────────────────────────────────────────
    private final GenericEntry e_overall;
    private final GenericEntry e_gyro;
    private final GenericEntry e_swerve;
    private final GenericEntry e_intake;
    private final GenericEntry e_intakeArm;
    private final GenericEntry e_uptake;
    private final GenericEntry e_hopperFeed;
    private final GenericEntry e_shooter;

    // ── Constructor ──────────────────────────────────────────────────────────

    public HardwareTestSuite() {
        ShuffleboardTab tab = Shuffleboard.getTab("Hardware Tests");

        e_overall    = tab.add("Overall Status",    PENDING).withPosition(0, 0).withSize(4, 1).getEntry();
        e_gyro       = tab.add("1. Gyro (Pigeon)",  PENDING).withPosition(0, 1).withSize(2, 1).getEntry();
        e_swerve     = tab.add("2. Swerve Drive",   PENDING).withPosition(2, 1).withSize(2, 1).getEntry();
        e_intake     = tab.add("3. Intake Motors",  PENDING).withPosition(0, 2).withSize(2, 1).getEntry();
        e_intakeArm  = tab.add("4. Intake Arm",     PENDING).withPosition(2, 2).withSize(2, 1).getEntry();
        e_uptake     = tab.add("5. Uptake",         PENDING).withPosition(0, 3).withSize(2, 1).getEntry();
        e_hopperFeed = tab.add("6. Hopper Feed",    PENDING).withPosition(2, 3).withSize(2, 1).getEntry();
        e_shooter    = tab.add("7. Shooter",        PENDING).withPosition(0, 4).withSize(2, 1).getEntry();

        tab.add("HOW TO USE",
                "1. Set Driver Station to TEST mode and enable.\n"
              + "2. Ensure robot has clearance to move ~10 cm forward.\n"
              + "3. Press OPERATOR START button to begin tests.\n"
              + "4. Watch this tab for PASS / FAIL results.")
           .withPosition(0, 5).withSize(4, 3);
    }

    // ── Public API ───────────────────────────────────────────────────────────

    /**
     * Builds the full sequential test command. A new command is built each
     * call so the sequence can be re-run within the same match.
     *
     * <p>Schedule this command when the operator presses the START button
     * (handled in Robot.testPeriodic).
     */
    public Command buildTestSequence() {
        return Commands.sequence(
            Commands.runOnce(this::resetAllStatus),
            testGyro(),
            Commands.waitSeconds(0.25),
            testSwerve(),
            Commands.waitSeconds(0.25),
            testIntake(),
            Commands.waitSeconds(0.25),
            testIntakeArm(),
            Commands.waitSeconds(0.25),
            testUptake(),
            Commands.waitSeconds(0.25),
            testHopperFeed(),
            Commands.waitSeconds(0.25),
            testShooter(),
            Commands.runOnce(this::finalizeOverall)
        );
    }

    // ── Helpers ──────────────────────────────────────────────────────────────

    private void resetAllStatus() {
        passCount = 0;
        failCount = 0;
        e_overall.setString("RUNNING TESTS...");
        e_gyro.setString(PENDING);
        e_swerve.setString(PENDING);
        e_intake.setString(PENDING);
        e_intakeArm.setString(PENDING);
        e_uptake.setString(PENDING);
        e_hopperFeed.setString(PENDING);
        e_shooter.setString(PENDING);
    }

    private void finalizeOverall() {
        if (failCount == 0) {
            e_overall.setString(String.format("ALL %d TESTS PASSED \u2713", passCount));
        } else {
            e_overall.setString(String.format("%d PASSED, %d FAILED \u2717", passCount, failCount));
        }
    }

    private void recordPass(GenericEntry entry, String detail) {
        passCount++;
        entry.setString(PASS + "  " + detail);
    }

    private void recordFail(GenericEntry entry, String detail) {
        failCount++;
        entry.setString(FAIL + "  " + detail);
    }

    // ── Individual Tests ─────────────────────────────────────────────────────

    /**
     * Test 1 – Gyro (Pigeon 2).
     * Reads yaw, pitch, and roll. No motion required.
     * Fails if any value is NaN (CAN error) or the robot is tilted more than
     * 45° (suggesting it is on its side or the sensor is wired backwards).
     */
    private Command testGyro() {
        return Commands.runOnce(() -> {
            e_gyro.setString(RUNNING);
            double yaw   = RobotContainer.gyro.getYawAngle();
            double pitch = RobotContainer.gyro.getPitchAngle();
            double roll  = RobotContainer.gyro.getRollAngle();

            if (Double.isNaN(yaw) || Double.isNaN(pitch) || Double.isNaN(roll)) {
                recordFail(e_gyro, "(NaN — check Pigeon CAN ID / wiring)");
            } else if (Math.abs(pitch) > 45.0 || Math.abs(roll) > 45.0) {
                recordFail(e_gyro, String.format("(excessive tilt: pitch=%.1f° roll=%.1f°)", pitch, roll));
            } else {
                recordPass(e_gyro, String.format("yaw=%.1f° pitch=%.1f° roll=%.1f°", yaw, pitch, roll));
            }
        });
    }

    /**
     * Test 2 – Swerve Drive.
     * Commands the robot forward at 0.3 m/s for 0.5 s (~15 cm). Reads the
     * average wheel velocity from all four drive motors.
     * <p><b>Robot will move — ensure clearance!</b>
     */
    private Command testSwerve() {
        final double[] maxVel = { 0.0 };
        return Commands.sequence(
            Commands.runOnce(() -> {
                e_swerve.setString(RUNNING);
                maxVel[0] = 0.0;
            }),
            Commands.runEnd(
                () -> {
                    RobotContainer.drivesystem.RobotDrive(0.3, 0.0, 0.0, false);
                    maxVel[0] = Math.max(maxVel[0],
                            RobotContainer.drivesystem.getAverageDriveVelocityRPS());
                },
                () -> RobotContainer.drivesystem.RobotDrive(0.0, 0.0, 0.0, false),
                RobotContainer.drivesystem
            ).withTimeout(0.5),
            Commands.runOnce(() -> {
                // At 0.3 m/s, expect ~1 wheel RPS. Pass threshold is 0.4 RPS.
                if (maxVel[0] >= 0.4) {
                    recordPass(e_swerve, String.format("peak avg %.2f wheel RPS", maxVel[0]));
                } else {
                    recordFail(e_swerve, String.format("only %.2f RPS — check drive motors / CAN", maxVel[0]));
                }
            })
        );
    }

    /**
     * Test 3 – Intake Motors.
     * Runs intake at normal speed for 0.75 s. Fails if the motor does not
     * spin up (broken CAN, tripped breaker, or mechanical jam).
     */
    private Command testIntake() {
        final double[] maxVel = { 0.0 };
        return Commands.sequence(
            Commands.runOnce(() -> {
                e_intake.setString(RUNNING);
                maxVel[0] = 0.0;
            }),
            Commands.runEnd(
                () -> {
                    RobotContainer.intake.intake();
                    maxVel[0] = Math.max(maxVel[0],
                            Math.abs(RobotContainer.intake.getVelocityRPS()));
                },
                () -> RobotContainer.intake.stop(),
                RobotContainer.intake
            ).withTimeout(0.75),
            Commands.runOnce(() -> {
                if (maxVel[0] >= 0.5) {
                    recordPass(e_intake, String.format("peak %.1f RPS", maxVel[0]));
                } else {
                    recordFail(e_intake,
                            String.format("only %.2f RPS — check motors/breaker", maxVel[0]));
                }
            })
        );
    }

    /**
     * Test 4 – Intake Arm.
     * Commands the arm to the STOWED (fully retracted) position using Motion
     * Magic. Checks that the encoder position converges within 2 rotations of
     * the target. Movement is always toward the safe end-stop.
     */
    private Command testIntakeArm() {
        final double[] startPos = { 0.0 };
        return Commands.sequence(
            Commands.runOnce(() -> {
                e_intakeArm.setString(RUNNING);
                startPos[0] = RobotContainer.intakeArm.currentPose;
            }),
            Commands.runEnd(
                () -> RobotContainer.intakeArm.moveTo(RobotMap.IntakeArm.STOWED_POSITION),
                () -> { /* hold commanded position */ },
                RobotContainer.intakeArm
            ).withTimeout(2.5)
             .until(() -> Math.abs(RobotContainer.intakeArm.currentPose
                                   - RobotMap.IntakeArm.STOWED_POSITION) < 0.5),
            Commands.runOnce(() -> {
                double err = Math.abs(RobotContainer.intakeArm.currentPose
                                      - RobotMap.IntakeArm.STOWED_POSITION);
                if (err < 2.0) {
                    recordPass(e_intakeArm,
                            String.format("pos=%.2f rot (err=%.2f)", RobotContainer.intakeArm.currentPose, err));
                } else {
                    recordFail(e_intakeArm,
                            String.format("err=%.2f rot (start=%.2f) — check encoder/motor", err, startPos[0]));
                }
            })
        );
    }

    /**
     * Test 5 – Uptake.
     * Runs the uptake motor for 0.75 s and checks velocity feedback.
     * Automatically SKIPPED when {@code ENABLE_LEFT_TURRET = false} because
     * the hardware is replaced by a software stub.
     */
    private Command testUptake() {
        if (!RobotMap.Features.ENABLE_LEFT_TURRET) {
            return Commands.runOnce(() -> e_uptake.setString(SKIPPED + " (ENABLE_LEFT_TURRET=false)"));
        }
        final double[] maxVel = { 0.0 };
        return Commands.sequence(
            Commands.runOnce(() -> {
                e_uptake.setString(RUNNING);
                maxVel[0] = 0.0;
            }),
            Commands.runEnd(
                () -> {
                    RobotContainer.uptake.feedShooter();
                    maxVel[0] = Math.max(maxVel[0],
                            Math.abs(RobotContainer.uptake.getVelocityRPS()));
                },
                () -> RobotContainer.uptake.stop(),
                RobotContainer.uptake
            ).withTimeout(0.75),
            Commands.runOnce(() -> {
                if (maxVel[0] >= 0.5) {
                    recordPass(e_uptake, String.format("peak %.1f RPS", maxVel[0]));
                } else {
                    recordFail(e_uptake,
                            String.format("only %.2f RPS — check motor/breaker", maxVel[0]));
                }
            })
        );
    }

    /**
     * Test 6 – Hopper Feed.
     * The HopperFeed subsystem drives its motor autonomously in
     * {@code periodic()} whenever the robot is enabled, so there is no
     * separate "run" command needed. This test simply waits half a second for
     * the motor to spin up and then reads the velocity.
     */
    private Command testHopperFeed() {
        return Commands.sequence(
            Commands.runOnce(() -> e_hopperFeed.setString(RUNNING)),
            Commands.waitSeconds(0.5),   // periodic() starts the motor automatically
            Commands.runOnce(() -> {
                double vel = Math.abs(RobotContainer.hopperFeed.getVelocityRPS());
                if (vel >= 0.5) {
                    recordPass(e_hopperFeed, String.format("%.1f RPS", vel));
                } else {
                    recordFail(e_hopperFeed,
                            String.format("only %.2f RPS — check motor/CAN/breaker", vel));
                }
            })
        );
    }

    /**
     * Test 7 – Shooter Flywheel.
     * Commands the flywheel to 20 RPS for 2 s and checks that it reaches at
     * least 10 RPS (50 % of target). A motor with a wiring fault or tripped
     * breaker will stay near 0.
     * <p>SKIPPED when {@code ENABLE_LEFT_TURRET = false}.
     */
    private Command testShooter() {
        if (!RobotMap.Features.ENABLE_LEFT_TURRET) {
            return Commands.runOnce(() -> e_shooter.setString(SKIPPED + " (ENABLE_LEFT_TURRET=false)"));
        }
        final double[] maxRPS = { 0.0 };
        return Commands.sequence(
            Commands.runOnce(() -> {
                e_shooter.setString(RUNNING);
                maxRPS[0] = 0.0;
            }),
            Commands.runEnd(
                () -> {
                    RobotContainer.shooter.shooterSpeed(20.0);
                    maxRPS[0] = Math.max(maxRPS[0], RobotContainer.shooter.velocity);
                },
                () -> RobotContainer.shooter.stop(),
                RobotContainer.shooter
            ).withTimeout(2.0),
            Commands.runOnce(() -> {
                if (maxRPS[0] >= 10.0) {
                    recordPass(e_shooter, String.format("peak %.1f RPS", maxRPS[0]));
                } else {
                    recordFail(e_shooter,
                            String.format("only %.1f RPS — check motor/breaker", maxRPS[0]));
                }
            })
        );
    }
}
