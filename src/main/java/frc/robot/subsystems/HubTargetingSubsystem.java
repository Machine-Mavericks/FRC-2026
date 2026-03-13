package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.utils.ShooterCalculations;
import frc.robot.utils.TargetCalculations;

/**
 * HubTargetingSubsystem
 *
 * Owns all real-time hub-targeting state computed from the Limelight.
 * This subsystem runs every scheduler loop (50 Hz) and provides:
 *
 * - Whether a valid HUB AprilTag is currently visible
 * - Horizontal angle error that needs to be corrected (= Limelight tx)
 * - Calculated distance to the HUB (from Limelight ty + known geometry)
 * - Shooter flywheel RPM appropriate for that distance
 *
 * AutoTrackGoal and (optionally) a future ShooterSubsystem read from this
 * class.
 *
 * ============================================================
 * TUNING CHECKLIST — do this before competition:
 * ============================================================
 * 1. Measure and update RobotMap.Vision.SHOOTER_LIMELIGHT_MOUNT_HEIGHT_M
 * 2. Measure and update RobotMap.Vision.SHOOTER_LIMELIGHT_MOUNT_ANGLE_DEG
 * 3. Update RobotMap.Vision.BLUE/RED_HUB_TAG_IDS if the manual's
 * field diagram shows different IDs than the placeholder arrays.
 * 4. Tune ShooterCalculations.DISTANCE_RPM_TABLE on the real robot.
 * 5. Adjust TARGET_TX_TOLERANCE_DEG if turret mechanical backlash
 * requires a looser or tighter threshold.
 * ============================================================
 */
public class HubTargetingSubsystem extends SubsystemBase {

    // How close (degrees) the horizontal angle error must be for "on target"
    private static final double TARGET_TX_TOLERANCE_DEG = 2.0;

    // ---- State updated each loop ----
    private boolean m_hubTagVisible = false;
    private int m_bestTagId = -1;
    private double m_txDegrees = 0.0; // horizontal angle error (degrees), + = target right
    private double m_tyDegrees = 0.0; // vertical angle (degrees), + = target above center
    private double m_distanceMeters = -1.0;
    private double m_shooterRPM = 0.0;
    private boolean m_isRedAlliance = false;

    // ---- Shuffleboard ----
    private GenericEntry m_sbTagVisible;
    private GenericEntry m_sbTagId;
    private GenericEntry m_sbTx;
    private GenericEntry m_sbTy;
    private GenericEntry m_sbDistance;
    private GenericEntry m_sbRPM;
    private GenericEntry m_sbOnTarget;

    public HubTargetingSubsystem() {
        initializeShuffleboard();
    }

    // -------------------------------------------------------------------------
    // Periodic — called 50 Hz by the scheduler
    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        // Determine alliance
        var alliance = DriverStation.getAlliance();
        m_isRedAlliance = alliance.isPresent() && alliance.get() == Alliance.Red;

        // // Read raw Limelight values
        // m_txDegrees = RobotContainer.limelightShooter.getHorizontalTargetOffsetAngle();
        // m_tyDegrees = RobotContainer.limelightShooter.getVerticalTargetOffsetAngle();
        // m_bestTagId = (int) RobotContainer.limelightShooter.getPrimAprilTagID();

        // Check whether what the camera sees is actually one of our HUB's tags
        boolean limelightHasTarget = RobotContainer.limelightShooter.isTargetPresent();
        m_hubTagVisible = limelightHasTarget
                && TargetCalculations.isOurHubTag(m_bestTagId, m_isRedAlliance);

        if (m_hubTagVisible) {
            // Compute distance from ty angle + known heights
            m_distanceMeters = ShooterCalculations.calculateDistanceFromTy(m_tyDegrees);

            // Compute shooter RPM
            if (m_distanceMeters > 0) {
                m_shooterRPM = ShooterCalculations.getShooterRPM(m_distanceMeters);
            } else {
                // Geometry invalid (camera below tag, or extreme angle): hold last value
                m_shooterRPM = 0.0;
            }
        } else {
            // No valid tag visible — clear real-time values but keep last distance/RPM
            // so AutoTrackGoal can still do a field-coordinate fallback calculation
            m_distanceMeters = -1.0;
            m_shooterRPM = 0.0;
        }

        updateShuffleboard();
    }

    // -------------------------------------------------------------------------
    // Public Getters — read by AutoTrackGoal and future ShooterSubsystem
    // -------------------------------------------------------------------------

    /**
     * @return True if the Limelight is currently tracking a valid HUB AprilTag
     *         for our alliance.
     */
    public boolean isHubTagVisible() {
        return m_hubTagVisible;
    }

    /**
     * @return The Limelight's horizontal angle error to the target (degrees).
     *         Positive = target is to the right of camera center.
     *         Negative = target is to the left.
     *         Zero when no tag is visible (use isHubTagVisible() to check).
     */
    public double getHorizontalAngleError() {
        return m_txDegrees;
    }

    /**
     * @return The Limelight's vertical angle to the target (degrees).
     *         Positive = target is above camera center.
     */
    public double getVerticalAngle() {
        return m_tyDegrees;
    }

    /**
     * @return Calculated straight-line distance to the HUB (meters).
     *         Returns -1 if no tag is visible or geometry is invalid.
     */
    public double getDistanceToHub() {
        return m_distanceMeters;
    }

    /**
     * @return Recommended shooter flywheel RPM for the current distance.
     *         Returns 0.0 if no valid target is visible.
     */
    public double getShooterRPM() {
        return m_shooterRPM;
    }

    /**
     * @return The AprilTag ID the Limelight has selected as its primary target.
     *         Returns -1 when no tag is detected.
     */
    public int getBestTagId() {
        return m_bestTagId;
    }

    /**
     * @return True when the horizontal angle error is within
     *         TARGET_TX_TOLERANCE_DEG.
     *         Use this (combined with turret atSetpoint()) to gate the shoot
     *         command.
     */
    public boolean isAimedAtHub() {
        return m_hubTagVisible
                && TargetCalculations.isOnTarget(m_txDegrees, TARGET_TX_TOLERANCE_DEG);
    }

    /**
     * @return True if we currently detect the alliance as red.
     */
    public boolean isRedAlliance() {
        return m_isRedAlliance;
    }

    // -------------------------------------------------------------------------
    // Shuffleboard
    // -------------------------------------------------------------------------

    private void initializeShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Hub Targeting");

        // Status column
        ShuffleboardLayout statusLayout = tab.getLayout("Status", BuiltInLayouts.kList)
                .withPosition(0, 0).withSize(2, 4);
        m_sbTagVisible = statusLayout.add("Hub Tag Visible", false).getEntry();
        m_sbTagId = statusLayout.add("Tag ID", -1).getEntry();
        m_sbOnTarget = statusLayout.add("Aimed at Hub", false).getEntry();

        // Limelight angles column
        ShuffleboardLayout anglesLayout = tab.getLayout("Camera Angles", BuiltInLayouts.kList)
                .withPosition(2, 0).withSize(2, 4);
        m_sbTx = anglesLayout.add("tx (horiz error, deg)", 0.0).getEntry();
        m_sbTy = anglesLayout.add("ty (vert angle, deg)", 0.0).getEntry();

        // Calculated values column
        ShuffleboardLayout calcLayout = tab.getLayout("Calculated", BuiltInLayouts.kList)
                .withPosition(4, 0).withSize(2, 4);
        m_sbDistance = calcLayout.add("Distance to Hub (m)", 0.0).getEntry();
        m_sbRPM = calcLayout.add("Shooter RPM", 0.0).getEntry();
    }

    private void updateShuffleboard() {
        m_sbTagVisible.setBoolean(m_hubTagVisible);
        m_sbTagId.setInteger(m_bestTagId);
        m_sbTx.setDouble(m_txDegrees);
        m_sbTy.setDouble(m_tyDegrees);
        m_sbDistance.setDouble(m_distanceMeters > 0 ? m_distanceMeters : 0.0);
        m_sbRPM.setDouble(m_shooterRPM);
        m_sbOnTarget.setBoolean(isAimedAtHub());
    }

}
