package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Uptake;
import frc.robot.subsystems.HopperFeed;

/**
 * Automates the process of shooting a ball by spinning the shooter flywheels
 * and waiting until they are at the correct speed to feed the intake arm and
 * uptake.
 */
public class ShootSequence extends Command {

    private final Shooter shooter;
    private final Uptake uptake;
    private final HopperFeed hopperFeed;

    /**
     * Creates a new ShootSequence.
     *
     * @param shooter    The shooter subsystem to run.
     * @param uptake     The uptake subsystem to feed balls into the shooter
     *                   flywheels.
     * @param hopperFeed The hopper feed subsystem to feed balls into the uptake.
     */
    public ShootSequence(Shooter shooter, Uptake uptake, HopperFeed hopperFeed) {
        this.shooter = shooter;
        this.uptake = uptake;
        this.hopperFeed = hopperFeed;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter, uptake, hopperFeed);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // We calculate and command the target RPM based on Limelight distance
        shooter.shooterSpeed(RobotContainer.autoTrack.getCalculatedShooterRPM());
        uptake.stop();
        hopperFeed.stop();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Continuously update the target RPM in case distance changes
        shooter.shooterSpeed(RobotContainer.autoTrack.getCalculatedShooterRPM());

        // Use autoTrack's built-in alignment and speed check to determine when to fire
        if (RobotContainer.autoTrack.isReadyToShoot()) {
            uptake.feedShooter();
            hopperFeed.feed();
        } else {
            uptake.stop();
            hopperFeed.stop();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        uptake.stop();
        hopperFeed.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // Tied to a whileTrue() binding (ends when button released)
    }
}
