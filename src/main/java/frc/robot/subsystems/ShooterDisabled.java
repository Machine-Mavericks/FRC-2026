package frc.robot.subsystems;

/**
 * Disabled shooter that extends the real Shooter class but overrides
 * actuation methods to avoid commanding hardware. This keeps type
 * compatibility with existing code while making it safe to run on robots
 * without shooter hardware.
 */
public class ShooterDisabled extends Shooter {

    public ShooterDisabled() {
        super(true); // skip hardware initialization
    }

    @Override
    public void shooterSpeed(double speed) {
        // Do not command hardware. Update internal fields for callers.
        this.commanded = speed;
        this.targetSpeed = speed;
        this.velocity = speed;
        this.wheelRPM = speed * 60.0;
    }

    @Override
    public void stop() {
        shooterSpeed(0.0);
    }

    @Override
    public double CalculateSpeed(double distance) {
        return 0.0;
    }
}

