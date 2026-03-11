package frc.robot.utils;

import frc.robot.RobotMap;

/**
 * Utility class for calculating shooter speed based on distance to the HUB
 * goal.
 *
 * Uses a lookup table with linear interpolation. All distance values represent
 * the straight-line distance from the robot to the HUB center.
 *
 * !! IMPORTANT: The RPM values in the lookup table are PLACEHOLDERS. !!
 * !! They must be tuned empirically on the actual robot with the real shooter!!
 */
public class ShooterCalculations {

    /**
     * Lookup table: distance (meters from robot to HUB center) → shooter flywheel
     * RPM.
     *
     * The HUB opening is 72" (1.828 m) above the carpet.
     * The HUB is 47"x47" (~1.19 m); a shot from the Alliance Zone travels roughly
     * 1–6 m.
     * Distances below 1.0 m or above 7.5 m are clamped to the nearest table entry.
     *
     * Tune these values:
     * - Set up the robot at each distance
     * - Increase RPM until shots consistently score
     * - Record the working RPM in the table below
     */
    private static final double[][] DISTANCE_RPM_TABLE = {
            // { distance (m), RPM }
            { 1.00, 2000.0 },
            { 1.50, 2200.0 },
            { 2.00, 2450.0 },
            { 2.50, 2700.0 },
            { 3.00, 2950.0 },
            { 3.50, 3200.0 },
            { 4.00, 3450.0 },
            { 4.50, 3700.0 },
            { 5.00, 3950.0 },
            { 5.50, 4200.0 },
            { 6.00, 4450.0 },
            { 6.50, 4700.0 },
            { 7.00, 4950.0 },
            { 7.50, 5200.0 },
    };

    /**
     * Get the target shooter speed (RPM) from a straight-line distance to the HUB
     * center.
     * Uses linear interpolation between lookup table entries.
     *
     * @param distanceMeters Distance from robot to HUB center in meters
     * @return Target flywheel speed in RPM
     */
    public static double getShooterRPM(double distanceMeters) {
        // Clamp below minimum
        if (distanceMeters <= DISTANCE_RPM_TABLE[0][0]) {
            return DISTANCE_RPM_TABLE[0][1];
        }
        // Clamp above maximum
        if (distanceMeters >= DISTANCE_RPM_TABLE[DISTANCE_RPM_TABLE.length - 1][0]) {
            return DISTANCE_RPM_TABLE[DISTANCE_RPM_TABLE.length - 1][1];
        }

        // Linear interpolation
        for (int i = 0; i < DISTANCE_RPM_TABLE.length - 1; i++) {
            double d1 = DISTANCE_RPM_TABLE[i][0];
            double d2 = DISTANCE_RPM_TABLE[i + 1][0];
            if (distanceMeters >= d1 && distanceMeters <= d2) {
                double rpm1 = DISTANCE_RPM_TABLE[i][1];
                double rpm2 = DISTANCE_RPM_TABLE[i + 1][1];
                double t = (distanceMeters - d1) / (d2 - d1);
                return rpm1 + t * (rpm2 - rpm1);
            }
        }

        // Fallback (should never reach here)
        return DISTANCE_RPM_TABLE[DISTANCE_RPM_TABLE.length / 2][1];
    }

    /**
     * Get the target shooter RPM from the Limelight's vertical angle (ty).
     *
     * This is the primary real-time method used when a HUB AprilTag is directly
     * visible.
     * It converts the camera's vertical angle to a distance using the triangle
     * formed by:
     * - Known AprilTag height above carpet
     * - Known camera mount height above carpet
     * - Known camera upward tilt angle
     *
     * Formula:
     * distance = (tagHeight - cameraHeight) / tan(mountAngle + ty)
     *
     * @param tyDegrees Vertical angle from Limelight (positive = target is above
     *                  center)
     * @return Target flywheel speed in RPM, or -1 if the geometry is invalid
     */
    public static double getShooterRPMFromTy(double tyDegrees) {
        double distance = calculateDistanceFromTy(tyDegrees);
        if (distance <= 0)
            return -1.0; // invalid geometry
        return getShooterRPM(distance);
    }

    /**
     * Calculate straight-line horizontal distance to the HUB AprilTag from a
     * Limelight ty reading.
     *
     * Uses constants from RobotMap.Vision. Update SHOOTER_LIMELIGHT_MOUNT_HEIGHT_M
     * and
     * SHOOTER_LIMELIGHT_MOUNT_ANGLE_DEG to match the physical robot before using
     * this
     * method.
     *
     * @param tyDegrees Vertical offset angle from Limelight (degrees, positive =
     *                  above)
     * @return Horizontal distance to the tag in meters, or -1 if geometry is
     *         invalid
     */
    public static double calculateDistanceFromTy(double tyDegrees) {
        double angleRad = Math.toRadians(
                RobotMap.Vision.SHOOTER_LIMELIGHT_MOUNT_ANGLE_DEG + tyDegrees);

        // Guard against negative or near-zero denominator (camera pointing down at tag)
        if (angleRad <= 0.01)
            return -1.0;

        double heightDelta = RobotMap.Vision.HUB_APRILTAG_HEIGHT_M
                - RobotMap.Vision.SHOOTER_LIMELIGHT_MOUNT_HEIGHT_M;

        if (heightDelta <= 0)
            return -1.0; // camera is above the tag — unusual

        return heightDelta / Math.tan(angleRad);
    }

    /** @return Minimum distance supported by the lookup table (meters) */
    public static double getMinDistance() {
        return DISTANCE_RPM_TABLE[0][0];
    }

    /** @return Maximum distance supported by the lookup table (meters) */
    public static double getMaxDistance() {
        return DISTANCE_RPM_TABLE[DISTANCE_RPM_TABLE.length - 1][0];
    }
}
