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
        super(RobotMap.CANID.RIGHT_TURRET_MOTOR, "Turret Right");
    }
}
