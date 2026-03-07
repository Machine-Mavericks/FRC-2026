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
        super(skipHardware, RobotMap.CANID.LEFT_TURRET_MOTOR, "Turret Left");
    }
}
