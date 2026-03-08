package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Uptake;

/**
 * Automates the process of shooting a ball by spinning the shooter flywheels
 * and waiting until they are at the correct speed to feed the intake arm and
 * uptake.
 */
public class ShootSequence extends Command {

    private final Shooter shooter;
    private final IntakeArm intakeArm;
    private final Uptake uptake;

    /**
     * Creates a new ShootSequence.
     *
     * @param shooter   The shooter subsystem to run.
     * @param intakeArm The intake arm subsystem to feed balls into the shooter.
     * @param uptake    The uptake subsystem to feed balls into the shooter flywheels.
     */
    public ShootSequence(Shooter shooter, IntakeArm intakeArm, Uptake uptake) {
        this.shooter = shooter;
        this.intakeArm = intakeArm;
        this.uptake = uptake;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter, intakeArm, uptake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // We calculate and command the target RPM based on Limelight distance
        shooter.shooterSpeed(RobotContainer.autoTrack.getCalculatedShooterRPM());
        intakeArm.stop(); // Stop intake arm initially while shooter spins up
        uptake.stop();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Do not spin up or feed if our alliance's hub is currently inactive —
        // scoring fuel would earn zero points.
        if (!RobotContainer.hubTargeting.isHubActive()) {
            shooter.stop();
            intakeArm.stop();
            uptake.stop();
            return;
        }

        // Continuously update the target RPM in case distance changes
        shooter.shooterSpeed(RobotContainer.autoTrack.getCalculatedShooterRPM());

        // Use autoTrack's built-in alignment and speed check to determine when to fire
        if (RobotContainer.autoTrack.isReadyToShoot()) {
            intakeArm.upIntakeArm(); // Feed exactly when ready
            uptake.feedShooter();
        } else {
            intakeArm.stop(); // Wait for flywheels/alignment
            uptake.stop();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        intakeArm.stop();
        uptake.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // Tied to a whileTrue() binding (ends when button released)
    }
}
