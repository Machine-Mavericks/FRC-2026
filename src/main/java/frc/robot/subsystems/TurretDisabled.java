package frc.robot.subsystems;

/**
 * Disabled subclass of the left turret. This still constructs the base
 * TurretLeft (and therefore its hardware object), but overrides actuation
 * methods to be safe no-ops. This keeps the class type compatible with the
 * rest of the code while preventing commands from driving absent hardware.
 */
public class TurretDisabled extends TurretLeft {

    private double targetAngleDegrees = 0.0;

    public TurretDisabled() {
        super(true); // construct base TurretLeft but skip hardware init
    }

    @Override
    public void setTargetAngle(double angleDegrees) {
        // Keep an internal target but do not command hardware
        this.targetAngleDegrees = angleDegrees;
    }

    @Override
    public double getCurrentAngle() {
        // Report the commanded target as current angle for stability
        return targetAngleDegrees;
    }

    @Override
    public boolean atSetpoint() {
        // Always treat disabled turret as "on target" so it doesn't block
        // shooting logic.
        return true;
    }

    @Override
    public void enableManualControl(boolean enabled) {
        // No-op
    }

    @Override
    public void setManualSpeed(double speed) {
        // No-op
    }

    @Override
    public void resetEncoder() {
        this.targetAngleDegrees = 0.0;
    }

    @Override
    public double getTargetAngle() {
        return targetAngleDegrees;
    }
}
