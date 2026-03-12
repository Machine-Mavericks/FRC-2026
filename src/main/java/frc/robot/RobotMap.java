package frc.robot;

/**
 * Mapping and creation of hardware on the robot
 */
public class RobotMap {

    /**
     * Inner class to hold CAN ID constants.
     */
    public static class CANID {

        // CAN IDs for Swerve Cancoders
        public static final int LF_CANCODER = 9;
        public static final int RF_CANCODER = 10;
        public static final int LR_CANCODER = 11;
        public static final int RR_CANCODER = 12;
        // CAN IDs for Steer Motors
        public static final int LF_STEER_MOTOR = 2;
        public static final int RF_STEER_MOTOR = 4;
        public static final int LR_STEER_MOTOR = 6;
        public static final int RR_STEER_MOTOR = 8;
        // CAN IDs for Drive Motors
        public static final int LF_DRIVE_MOTOR = 1;
        public static final int RF_DRIVE_MOTOR = 3;
        public static final int LR_DRIVE_MOTOR = 5;
        public static final int RR_DRIVE_MOTOR = 7;

        // CAN ID for CTR Pigeon Gyro
        public static final int PIGEON = 14;

        // CAN IDs for Turret Motors (NEO 550 with SparkMax)
        public static final int LEFT_TURRET_MOTOR = 15;
        public static final int RIGHT_TURRET_MOTOR = 16;

        // Shooter motor
        public static final int SHOOTER = 20;

        // IntakeArm motors
        public static final int INTAKE_ARM_RIGHT = 23;
        public static final int INTAKE_ARM_LEFT = 24;

        // Intake motors
        public static final int INTAKE_SPIN = 28;

        // Uptake motors
        public static final int UPTAKE_MASTER = 25;
        public static final int UPTAKE_FOLLOWER = 26;

        // HopperFeed motor
        public static final int HOPPER_FEED = 27;

    }

    /**
     * Feature flags for enabling/disabling hardware or features at runtime.
     * Toggle these while testing or for robots that don't have certain hardware
     * installed.
     */
    public static class Features {
        /**
         * When false, the left turret (and related shooter/uptake hardware) is
         * disabled and replaced with software no-op stubs. Set to true to enable
         * the left turret and the shooter/uptake hardware.
         */
        public static final boolean ENABLE_LEFT_TURRET = false;
    }

    /**
     * Inner class to hold RoboRIO I/O connection constants
     */
    public static class DIO {
        // Deadwheel odometry pods

        // public static final int LEFTENCODER_A = 1;
        // public static final int LEFTENCODER_B = 0;
        // public static final int FRONTENCODER_A = 4;
        // public static final int FRONTENCODER_B = 5;
        // public static final int REARENCODER_A = 3;
        // public static final int REARENCODER_B = 2;
        public static final int photoSensor = 9;

    }

    public static class PWMPorts {
        /** PWM Port for led strip */

        // PWM port for camera tilting subsystem
        // public static final int CAMERA_SERVO_ID = 1;
    }

    /**
     * Inner class to hold RoboRIO analog input constants
     */
    public static class AINPorts {

        // IDs for NEO Swerve steer position sensors
        public static final int NEO_LF_STEER_SENSOR = 0;
        public static final int NEO_RF_STEER_SENSOR = 1;
        public static final int NEO_LR_STEER_SENSOR = 2;
        public static final int NEO_RR_STEER_SENSOR = 3;

    }

    // game controller port IDs
    public static class GamePadPorts {
        public static final int DriverID = 0;
        public static final int OperatorID = 1;
    }

    /**
     * Turret subsystem constants
     */
    public static class Turret {
        // Rotation limits (degrees from forward position)
        public static final double MAX_ROTATION_DEGREES = 90.0;
        public static final double MIN_ROTATION_DEGREES = -90.0;

        // PID gains for turret position control
        public static final double kP = 0.05;
        public static final double kI = 0.0;
        public static final double kD = 0.001;
        public static final double kIZone = 0.0;
        public static final double kFF = 0.0;

        // Position tolerance (degrees)
        public static final double POSITION_TOLERANCE = 2.0;

        // Manual control speed limit (percent output)
        public static final double MANUAL_SPEED_LIMIT = 0.1;// was 0.3

        // Gear ratio (motor rotations per turret rotation)
        public static final double GEAR_RATIO = 100.0; // Adjust based on actual mechanism

        // Turret offsets from robot center (meters) - adjust based on CAD
        public static final double LEFT_TURRET_X_OFFSET = -0.15; // Left of center
        public static final double LEFT_TURRET_Y_OFFSET = 0.20; // Forward of center
        public static final double RIGHT_TURRET_X_OFFSET = 0.15; // Right of center
        public static final double RIGHT_TURRET_Y_OFFSET = 0.20; // Forward of center
    }

    /**
     * Limelight and Vision constants
     */
    public static class Vision {

        // Limelight network table names (must match the names configured in the
        // cameras)
        public static final String LIMELIGHT_DRIVE_NAME = "limelight-drive";
        public static final String LIMELIGHT_SHOOTER_NAME = "limelight-shooter";

        // --------------- HUB AprilTag IDs (2026 Game Manual, p.34) ---------------
        // AprilTags are placed on all 4 faces of each alliance HUB.
        // Blue alliance HUB tags (confirm exact IDs from field layout diagram p.35)
        public static final int[] BLUE_HUB_TAG_IDS = { 18, 19, 20, 21, 24, 25, 26, 27 };
        // Red alliance HUB tags
        public static final int[] RED_HUB_TAG_IDS = { 2, 3, 4, 5, 8, 9, 10, 11 };

        // Legacy single-tag constants (kept for backward compatibility)
        public static final int BLUE_GOAL_TAG_ID = 9;
        public static final int RED_GOAL_TAG_ID = 10;

        // --------------- HUB Field Coordinates (meters, WPI Blue-origin)
        // ---------------
        // Each HUB center is ~158.6" (4.03 m) from its respective Alliance Wall.
        // Field width: ~317.7" (8.07 m), HUBs are centered at Y ≈ 4.04 m.
        // TODO: confirm exact values from field CAD / manual p.35-36
        public static final double BLUE_GOAL_X = 4.03; // ~158.6" from blue wall
        public static final double BLUE_GOAL_Y = 4.04; // center of field width
        public static final double RED_GOAL_X = 12.51; // field_len(~16.54) - 4.03
        public static final double RED_GOAL_Y = 4.04;

        // --------------- Neutral Zone Constants ---------------
        // Based on a ~16.54m field, Neutral Zone is approximately the middle area.
        public static final double NEUTRAL_ZONE_X_MIN = 5.50; // Approximated
        public static final double NEUTRAL_ZONE_X_MAX = 11.04; // Approximated

        // Where to aim if shooting from the neutral zone (to avoid hitting the hub)
        public static final double BLUE_SAFE_TARGET_X = 2.0; // Deep in blue territory
        public static final double BLUE_SAFE_TARGET_Y = 4.04; // Center
        public static final double RED_SAFE_TARGET_X = 14.54; // Deep in red territory
        public static final double RED_SAFE_TARGET_Y = 4.04; // Center

        // --------------- Physical Height Constants (2026 Game Manual, p.22 & p.34)
        // ---------------
        /** HUB opening (top of hexagonal goal) height above carpet: 72" */
        public static final double HUB_OPENING_HEIGHT_M = 1.828;
        /** Center of HUB AprilTags above carpet: 44.25" */
        public static final double HUB_APRILTAG_HEIGHT_M = 1.124;
        /** HUB hexagonal opening diameter: 41.7" */
        public static final double HUB_OPENING_DIAMETER_M = 1.060;

        // --------------- Limelight Mount (MEASURE ON YOUR ROBOT) ---------------
        /**
         * Height of Shooter Limelight lens center above carpet (meters) — measure on
         * actual
         * robot
         */
        public static final double SHOOTER_LIMELIGHT_MOUNT_HEIGHT_M = 0.60;
        /**
         * Upward tilt angle of Shooter Limelight from horizontal (degrees) — measure on
         * actual
         * robot
         */
        public static final double SHOOTER_LIMELIGHT_MOUNT_ANGLE_DEG = 30.0;

        // --------------- Limelight 3D Poses (OVERRIDE WEB UI) ---------------
        // These arrays configure the physical location of the cameras relative
        // to the center of the robot. This overrides the "botpose" settings in the Web
        // UI.
        // Format: [Forward, Side, Up, Roll, Pitch, Yaw]
        // Forward/Side/Up are in meters. Roll/Pitch/Yaw are in degrees.

        /**
         * 3D Pose of the Shooter Limelight relative to robot center.
         * Using the existing height and angle constants for Up and Pitch.
         * TODO: Measure precise Forward and Side offsets!
         */
        public static final double[] SHOOTER_LIMELIGHT_3D_POSE = {
                0.0, // Forward
                0.0, // Side
                SHOOTER_LIMELIGHT_MOUNT_HEIGHT_M, // Up
                0.0, // Roll
                SHOOTER_LIMELIGHT_MOUNT_ANGLE_DEG, // Pitch
                0.0 // Yaw
        };

        /**
         * 3D Pose of the Drive Limelight relative to robot center.
         * TODO: Entire Drive Limelight position is TBD. Leaving entirely as 0 for now.
         */
        public static final double[] DRIVE_LIMELIGHT_3D_POSE = {
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        };
    }

    /**
     * Intake subsystem constants
     */
    public static class Intake {
        public static final double INTAKE_SPEED = 0.8; // was 0.8
        public static final double OUTTAKE_SPEED = -0.5; // was -0.5
    }

    /**
     * IntakeArm subsystem constants
     */
    public static class IntakeArm {
        public static final double INTAKE_ARM_SPEED = 0.2;// was 0.5
        /**
         * Maximum arm position in mechanism rotations (0 degrees from horizontal)
         * (fully
         * deployed/down) — TUNE
         */
        public static final double FORWARD_SOFT_LIMIT = -5.0 / 360.0;
        /**
         * Minimum arm position in mechanism rotations (-90 degrees from horizontal)
         * (fully retracted/up)
         * — TUNE
         */
        public static final double REVERSE_SOFT_LIMIT = -90.0 / 360.0;
        /** Stator current limit (amps) to protect motor from stall damage */
        public static final double STATOR_CURRENT_LIMIT = 20.0;

        /** Target position (rotations) when arm is deployed for intaking — TUNE */
        public static final double DEPLOYED_POSITION = FORWARD_SOFT_LIMIT;
        /** Target position (rotations) when arm is stowed after intaking — TUNE */
        public static final double STOWED_POSITION = REVERSE_SOFT_LIMIT;
    }

    /**
     * Uptake subsystem constants
     */
    public static class Uptake {
        public static final double UPTAKE_SPEED = 0.8;
        public static final double GEAR_RATIO = 9.0;
    }

    /**
     * HopperFeed subsystem constants
     */
    public static class HopperFeed {
        /** Default feed speed (percent output, 0.0 – 1.0). Tunable via Shuffleboard. */
        public static final double DEFAULT_SPEED = 0.0;// was 0.5
    }
}