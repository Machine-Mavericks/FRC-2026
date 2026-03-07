package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Uptake;

/**
 * Automates the process of shooting a ball by spinning the shooter flywheels
 * and waiting until they are at the correct speed to feed the hopper and
 * uptake.
 */
public class ShootSequence extends Command {

    private final Shooter shooter;
    private final Hopper hopper;
    private final Uptake uptake;

    /**
     * Creates a new ShootSequence.
     *
     * @param shooter The shooter subsystem to run.
     * @param hopper  The hopper subsystem to feed balls into the shooter.
     * @param uptake  The uptake subsystem to feed balls into the shooter flywheels.
     */
    public ShootSequence(Shooter shooter, Hopper hopper, Uptake uptake) {
        this.shooter = shooter;
        this.hopper = hopper;
        this.uptake = uptake;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter, hopper, uptake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // We calculate and command the target RPM based on Limelight distance
        shooter.shooterSpeed(RobotContainer.autoTrack.getCalculatedShooterRPM());
        hopper.stop(); // Stop hopper initially while shooter spins up
        uptake.stop();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Continuously update the target RPM in case distance changes
        shooter.shooterSpeed(RobotContainer.autoTrack.getCalculatedShooterRPM());

        // Use autoTrack's built-in alignment and speed check to determine when to fire
        if (RobotContainer.autoTrack.isReadyToShoot()) {
            hopper.upHopper(); // Feed exactly when ready
            uptake.feedShooter();
        } else {
            hopper.stop(); // Wait for flywheels/alignment
            uptake.stop();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        hopper.stop();
        uptake.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // Tied to a whileTrue() binding (ends when button released)
    }
}
