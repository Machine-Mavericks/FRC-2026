package frc.robot.subsystems;

/**
 * Disabled uptake that extends the real Uptake but overrides actuation to
 * avoid driving hardware while keeping the same type.
 */
public class UptakeDisabled extends Uptake {

    public UptakeDisabled() {
        super(true);
    }

    @Override
    public void feedShooter() {
        // No-op
    }

    @Override
    public void stop() {
        // No-op
    }
}

