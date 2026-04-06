package frc.robot.utils;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

/**
 * Utility class for calculating targeting information for turrets and the HUB
 * goal.
 *
 * Two complementary approaches are supported:
 *
 * 1. **Field-coordinate mode** – uses robot odometry pose and known HUB field
 * coordinates.
 * Works even when no AprilTag is in view. Accuracy depends on odometry quality.
 *
 * 2. **Direct Limelight mode** – uses raw tx/ty angles from the camera for
 * real-time
 * turret correction and distance calculation. Requires a HUB AprilTag to be
 * visible.
 * This mode is lower latency and more accurate when a tag is in frame.
 */
public class TargetCalculations {

    // -------------------------------------------------------------------------
    // HUB visibility helpers
    // -------------------------------------------------------------------------

    /**
     * Returns true if the given AprilTag ID belongs to the blue alliance HUB.
     *
     * @param tagID ID returned by the Limelight
     */
    public static boolean isBluHubTag(int tagID) {
        for (int id : RobotMap.Vision.BLUE_HUB_TAG_IDS) {
            if (id == tagID)
                return true;
        }
        return false;
    }

    /**
     * Returns true if the given AprilTag ID belongs to the red alliance HUB.
     *
     * @param tagID ID returned by the Limelight
     */
    public static boolean isRedHubTag(int tagID) {
        for (int id : RobotMap.Vision.RED_HUB_TAG_IDS) {
            if (id == tagID)
                return true;
        }
        return false;
    }

    /**
     * Returns true if the given tag ID belongs to the alliance HUB we're trying to
     * score in.
     *
     * @param tagID         ID returned by the Limelight
     * @param isRedAlliance True when the robot is on the red alliance
     */
    public static boolean isOurHubTag(int tagID, boolean isRedAlliance) {
        return isRedAlliance ? isRedHubTag(tagID) : isBluHubTag(tagID);
    }

    // -------------------------------------------------------------------------
    // Field-coordinate mode (odometry-based)
    // -------------------------------------------------------------------------

    /**
     * Convert a point on the robot from robot-relative space to the corresponding point2D in field space
     * @param robotPose Current pose of the robot on the field
     * @param offsetX X offset from robot center (m, positive =
     *                      robot-right)
     * @param offsetY Y offset from robot center (m, positive =
     *                      robot-forward)     * @param offsetY
     * @return Corrseponding point on the field, with robot heading
     */
    public static Pose2d robotPointToFieldPoint(Pose2d robotPose, double offsetX, double offsetY){
        Translation2d robotTranslation = robotPose.getTranslation();
        Rotation2d robotRotation = robotPose.getRotation();

        // Transform turret offset from robot-relative to field-relative
        double fieldX = robotTranslation.getX()
                + offsetX * Math.cos(robotRotation.getRadians())
                - offsetY * Math.sin(robotRotation.getRadians());
        double fieldY = robotTranslation.getY()
                + offsetX * Math.sin(robotRotation.getRadians())
                + offsetY * Math.cos(robotRotation.getRadians());

        return new Pose2d(fieldX, fieldY, robotRotation);
    }

    /**
     * Calculate the angle a turret needs to rotate to point at the HUB goal.
     *
     * Uses field coordinates (robot pose + HUB position).
     *
     * @param robotPose     Current pose of the robot on the field
     * @param turretPose    Current pose of the turret on the field
     * @param goalPose      Position of the goal on the field
     * @return Target angle in degrees (0 = forward, positive = counterclockwise)
     */
    public static double getTargetAngleForTurret(
            Pose2d robotPose,
            Pose2d turretPose,
            Pose2d goalPose) {

            
        Rotation2d robotRotation = robotPose.getRotation();
        
        double fieldTurretX = turretPose.getX();
        double fieldTurretY = turretPose.getY();

        // Vector from turret position to goal
        double deltaX = goalPose.getX() - fieldTurretX;
        double deltaY = goalPose.getY() - fieldTurretY;

        // Absolute angle to goal in field coordinates
        double fieldAngleToGoal = Math.atan2(deltaY, deltaX);

        // Convert to robot-relative angle
        double robotRelativeAngle = fieldAngleToGoal - robotRotation.getRadians() - Rotation2d.fromDegrees(RobotMap.Turret.TURRET_ANGLE_OFFSET).getRadians();

        // Normalize to [-PI, PI]
        while (robotRelativeAngle > Math.PI)
            robotRelativeAngle -= 2 * Math.PI;
        while (robotRelativeAngle < -Math.PI)
            robotRelativeAngle += 2 * Math.PI;

        return Math.toDegrees(robotRelativeAngle);
    }

    /**
     * Calculate the straight-line distance from the robot to the HUB goal.
     *
     * @param robotPose Current pose of the robot
     * @param goalPose  Position of the goal center
     * @return Distance in meters
     */
    public static double getDistanceToGoal(Pose2d robotPose, Pose2d goalPose) {
        return robotPose.getTranslation().getDistance(goalPose.getTranslation());
    }

    /** @return HUB goal Pose2d for the blue alliance */
    public static Pose2d getBlueGoalPose() {
        return new Pose2d(RobotMap.Vision.BLUE_GOAL_X, RobotMap.Vision.BLUE_GOAL_Y, new Rotation2d());
    }

    /** @return HUB goal Pose2d for the red alliance */
    public static Pose2d getRedGoalPose() {
        return new Pose2d(RobotMap.Vision.RED_GOAL_X, RobotMap.Vision.RED_GOAL_Y, new Rotation2d());
    }

    /**
     * Get the HUB goal Pose2d for the current alliance.
     *
     * @param isRedAlliance True if on the red alliance
     * @return Pose2d of the appropriate HUB center
     */
    public static Pose2d getTargetGoalPose(boolean isRedAlliance) {
        return isRedAlliance ? getRedGoalPose() : getBlueGoalPose();
    }

    /**
     * Get one expected HUB AprilTag ID for the current alliance (used for basic tag
     * checking).
     *
     * @param isRedAlliance True if on the red alliance
     * @return A representative expected AprilTag ID
     */
    public static int getTargetTagID(boolean isRedAlliance) {
        return isRedAlliance ? RobotMap.Vision.RED_GOAL_TAG_ID : RobotMap.Vision.BLUE_GOAL_TAG_ID;
    }

    /**
     * Determine if the robot is currently in the neutral zone based on its pose.
     *
     * @param robotPose Current pose of the robot on the field
     * @return True if the robot is within the defined neutral zone X coordinates
     */
    public static boolean isInNeutralZone(Pose2d robotPose, boolean isRedAlliance) {
        Pose2d decisionPose = AutoFunctions.redVsBlue(new Pose2d(5.229, 4, Rotation2d.kZero));

        if (isRedAlliance){
            return robotPose.getX() < decisionPose.getX();
        } else {
            return robotPose.getX() > decisionPose.getX();
        }
    }

    /**
     * Calculate the turret angle needed to point to our alliance's "safe area"
     * when we are in the neutral zone.
     *
     * @param robotPose     Current pose of the robot
     * @param TurretPose    Current pose of the turret on the field
     * @param isRedAlliance True if on the red alliance
     * @return Target angle in degrees for the turret
     */
    public static double calculateNeutralZoneAimAngle(
            Pose2d robotPose,
            Pose2d TurretPose,
            boolean isRedAlliance) {
        Pose2d safeTarget = isRedAlliance ? getRedSafeTargetPose() : getBlueSafeTargetPose();
        return getTargetAngleForTurret(robotPose, TurretPose, safeTarget);
    }

    /** @return Safe target Pose2d for the blue alliance when in neutral zone */
    public static Pose2d getBlueSafeTargetPose() {
        return new Pose2d(RobotMap.Vision.BLUE_SAFE_TARGET_X, RobotMap.Vision.BLUE_SAFE_TARGET_Y, new Rotation2d());
    }

    /** @return Safe target Pose2d for the red alliance when in neutral zone */
    public static Pose2d getRedSafeTargetPose() {
        return new Pose2d(RobotMap.Vision.RED_SAFE_TARGET_X, RobotMap.Vision.RED_SAFE_TARGET_Y, new Rotation2d());
    }

    // -------------------------------------------------------------------------
    // Direct Limelight mode (camera-angle-based, requires tag in view)
    // -------------------------------------------------------------------------

    /**
     * Calculate the horizontal distance to the HUB AprilTag using the Limelight's
     * vertical offset angle (ty) and known geometry.
     *
     * This is the same calculation as
     * {@link ShooterCalculations#calculateDistanceFromTy}
     * but exposed here for use in turret-angle correction contexts.
     *
     * @param tyDegrees Limelight ty (degrees, positive = target above camera
     *                  center)
     * @return Horizontal distance in meters, or -1 if geometry is invalid
     */
    public static double calculateDistanceFromTy(double tyDegrees) {
        return ShooterCalculations.calculateDistanceFromTy(tyDegrees);
    }

    /**
     * Determine if the limelight tx offset (horizontal angle error) is small enough
     * that the turret can be considered "on target".
     *
     * @param txDegrees    Limelight tx value (degrees, positive = target to the
     *                     right)
     * @param toleranceDeg Acceptable angular error in degrees
     * @return True if the estimated error is within tolerance
     */
    public static boolean isOnTarget(double txDegrees, double toleranceDeg) {
        return Math.abs(txDegrees) <= toleranceDeg;
    }


    public static Pose2d getTargetPose(Pose2d robotPose) {
       boolean isRedAlliance = false;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            isRedAlliance = (alliance.get() == Alliance.Red);
        }

        boolean passing = isInNeutralZone(robotPose, isRedAlliance);

        if (passing) {
            if (isRedAlliance)
                return getRedSafeTargetPose();
            else
                return getBlueSafeTargetPose();
        } else {
            return TargetCalculations.getTargetGoalPose(isRedAlliance);
        }
    }
}
