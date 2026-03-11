package frc.robot.subsystems;

import frc.robot.RobotMap;

/**
 * Left turret subsystem.
 * Extends the base TurretSubsystem with left-specific configuration.
 */
public class TurretLeft extends TurretSubsystem {
    
    /**
     * Create the left turret subsystem.
     */
    public TurretLeft() {
        this(false);
    }

    /**
     * Create the left turret subsystem, optionally skipping hardware init.
     */
    public TurretLeft(boolean skipHardware) {
        // Calls the parent (TurretSubsystem) constructor.
        // - The first argument is the CAN ID for the right turret motor.
        // - The second argument is a human-readable name used for logging/diagnostics.
        // Note: a constructor must call super(...) as its first statement if it delegates to the base class.
        super(skipHardware, RobotMap.CANID.LEFT_TURRET_MOTOR, "Turret Left");
    }
}
