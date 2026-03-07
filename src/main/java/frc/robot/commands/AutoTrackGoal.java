package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.Limelight.PoseEstimate;
import frc.robot.utils.ShooterCalculations;
import frc.robot.utils.TargetCalculations;

/**
 * AutoTrackGoal
 *
 * Default command for both turrets. Runs continuously (never finishes) and
 * automatically points the turrets at the HUB while calculating shooter RPM.
 *
 * Priority logic (evaluated each loop):
 *
 * 1. DIRECT LIMELIGHT mode — A valid HUB AprilTag is in view.
 * -> Turret correction = tx from HubTargetingSubsystem (camera sees target
 * directly)
 * -> Shooter RPM = from HubTargetingSubsystem (distance via ty trig)
 * This gives the lowest latency and highest accuracy.
 *
 * 2. FIELD-COORDINATE mode — No tag visible, but robot pose is known.
 * -> Turret angle = calculated from robot odometry + known HUB coordinates
 * -> Shooter RPM = from distance between robot pose and HUB coordinates
 * Falls back to last known odometry pose if pose estimate is momentarily
 * unavailable.
 *
 * The command exposes getCalculatedShooterRPM() and isReadyToShoot() so a
 * future
 * ShooterSubsystem can gate firing on those conditions.
 */
public class AutoTrackGoal extends Command {

    private Pose2d lastRobotPose = null;
    private double calculatedShooterRPM = 0.0;
    private boolean targetVisible = false;

    public AutoTrackGoal() {
        addRequirements(RobotContainer.turretLeft, RobotContainer.turretRight);
    }

    @Override
    public void initialize() {
        RobotContainer.turretLeft.enableManualControl(false);
        RobotContainer.turretRight.enableManualControl(false);
        SmartDashboard.putString("Turret Mode", "Auto Tracking");
    }

    @Override
    public void execute() {

        // ------------------------------------------------------------------
        // Priority 1: Direct Limelight mode — AprilTag in view
        // ------------------------------------------------------------------
        if (RobotContainer.hubTargeting.isHubTagVisible()) {
            targetVisible = true;

            // tx is the horizontal angle error from camera center to tag center.
            // We send this directly as a correction delta to each turret's current
            // position.
            double txError = RobotContainer.hubTargeting.getHorizontalAngleError();

            double leftTarget = RobotContainer.turretLeft.getCurrentAngle() + txError;
            double rightTarget = RobotContainer.turretRight.getCurrentAngle() + txError;

            RobotContainer.turretLeft.setTargetAngle(leftTarget);
            RobotContainer.turretRight.setTargetAngle(rightTarget);

            // RPM from ty-based distance calculation
            calculatedShooterRPM = RobotContainer.hubTargeting.getShooterRPM();

            SmartDashboard.putString("Tracking Mode", "Limelight Direct");
            SmartDashboard.putNumber("Tx Error (deg)", txError);
            SmartDashboard.putNumber("Hub Distance (m)", RobotContainer.hubTargeting.getDistanceToHub());

            // ------------------------------------------------------------------
            // Priority 2: Field-coordinate mode — use odometry
            // ------------------------------------------------------------------
        } else {
            Pose2d robotPose = getRobotPoseFromOdometry();

            if (robotPose == null) {
                // Nothing we can do — hold current position
                targetVisible = false;
                SmartDashboard.putString("Tracking Mode", "No Pose");
                SmartDashboard.putBoolean("Target Visible", false);
                return;
            }

            lastRobotPose = robotPose;
            targetVisible = false; // tag not confirmed visible

            // Determine which HUB to aim at
            boolean isRedAlliance = false;
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                isRedAlliance = (alliance.get() == Alliance.Red);
            }

            Pose2d goalPose = TargetCalculations.getTargetGoalPose(isRedAlliance);

            // Calculate angles from robot pose + turret offsets + goal position
            double leftTurretAngle = TargetCalculations.getTargetAngleForTurret(
                    robotPose,
                    RobotMap.Turret.LEFT_TURRET_X_OFFSET,
                    RobotMap.Turret.LEFT_TURRET_Y_OFFSET,
                    goalPose);

            double rightTurretAngle = TargetCalculations.getTargetAngleForTurret(
                    robotPose,
                    RobotMap.Turret.RIGHT_TURRET_X_OFFSET,
                    RobotMap.Turret.RIGHT_TURRET_Y_OFFSET,
                    goalPose);

            double distance = TargetCalculations.getDistanceToGoal(robotPose, goalPose);
            calculatedShooterRPM = ShooterCalculations.getShooterRPM(distance);

            RobotContainer.turretLeft.setTargetAngle(leftTurretAngle);
            RobotContainer.turretRight.setTargetAngle(rightTurretAngle);

            SmartDashboard.putString("Tracking Mode", "Field Coordinates");
            SmartDashboard.putNumber("Tx Error (deg)", 0.0);
            SmartDashboard.putNumber("Hub Distance (m)", distance);
            SmartDashboard.putNumber("Left Turret Target (deg)", leftTurretAngle);
            SmartDashboard.putNumber("Right Turret Target (deg)", rightTurretAngle);
        }

        // ---- Shared telemetry ----
        SmartDashboard.putBoolean("Target Visible", targetVisible);
        SmartDashboard.putNumber("Calculated Shooter RPM", calculatedShooterRPM);
        SmartDashboard.putBoolean("Left Turret Ready", RobotContainer.turretLeft.atSetpoint());
        SmartDashboard.putBoolean("Right Turret Ready", RobotContainer.turretRight.atSetpoint());
        SmartDashboard.putBoolean("Ready to Shoot", isReadyToShoot());
    }

    /**
     * Get the robot pose from Limelight vision (MegaTag2), falling back to
     * odometry,
     * then to the last known pose. Used only in field-coordinate mode.
     */
    private Pose2d getRobotPoseFromOdometry() {
        // Try vision-based pose first (highest accuracy)
        if (RobotContainer.limelight.isTargetPresent()) {
            PoseEstimate estimate = RobotContainer.limelight.getBotPoseOrbDetail();
            if (estimate != null && estimate.pose != null) {
                return estimate.pose.toPose2d();
            }
        }

        // Try odometry
        if (RobotContainer.odometry != null) {
            return RobotContainer.odometry.getPose2d();
        }

        // Last resort — last known pose (robot moved since, but better than nothing)
        return lastRobotPose;
    }

    /**
     * Returns the shooter RPM calculated this loop.
     * A future ShooterSubsystem can call this to set its flywheel target.
     *
     * @return Shooter speed in RPM (0.0 if no valid target)
     */
    public double getCalculatedShooterRPM() {
        return calculatedShooterRPM;
    }

    /**
     * True when both turrets are at their setpoints AND either:
     * - A valid HUB tag is visible and we're aimed at it (Limelight mode), or
     * - We have a valid field-coordinate calculation (fallback mode)
     */
    public boolean isReadyToShoot() {
        boolean turretsOnTarget = RobotContainer.turretLeft.atSetpoint()
                && RobotContainer.turretRight.atSetpoint();

        if (RobotContainer.hubTargeting.isHubTagVisible()) {
            // In direct mode, also require that the camera confirms we're aimed correctly
            return turretsOnTarget && RobotContainer.hubTargeting.isAimedAtHub();
        }

        // In field-coordinate mode, just check turret setpoints
        return turretsOnTarget && lastRobotPose != null;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Turret Mode", "Stopped");
        SmartDashboard.putString("Tracking Mode", "Stopped");
        SmartDashboard.putBoolean("Ready to Shoot", false);
    }

    @Override
    public boolean isFinished() {
        return false; // This is a default command — runs continuously
    }
}
