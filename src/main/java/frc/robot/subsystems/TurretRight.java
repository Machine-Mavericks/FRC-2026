package frc.robot.subsystems;

import frc.robot.RobotMap;

/**
 * Right turret subsystem.
 * Extends the base TurretSubsystem with right-specific configuration.
 */
public class TurretRight extends TurretSubsystem {
    
    /**
     * Create the right turret subsystem.
     */
    public TurretRight() {
        // Calls the parent (TurretSubsystem) constructor.
        // - The first argument is the CAN ID for the right turret motor.
        // - The second argument is a human-readable name used for logging/diagnostics.
        // Note: a constructor must call super(...) as its first statement if it delegates to the base class.
        super(RobotMap.CANID.RIGHT_TURRET_MOTOR, "Turret Right");
    }
}
