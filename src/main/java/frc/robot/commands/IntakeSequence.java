package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Automates the process of intaking a ball and moving it into the hopper.
 */
public class IntakeSequence extends Command {

    private final IntakeSubsystem intake;
    private final Hopper hopper;

    /**
     * Creates a new IntakeSequence.
     *
     * @param intake The intake subsystem to run.
     * @param hopper The hopper subsystem to run.
     */
    public IntakeSequence(IntakeSubsystem intake, Hopper hopper) {
        this.intake = intake;
        this.hopper = hopper;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake, hopper);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intake.intake();
        hopper.upHopper(); // Run hopper slowly to pull the ball in
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Here we could add logic to stop if a limit switch/beam break is triggered in
        // the hopper.
        // For now, it will just continuously run while the button is held.
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.stop();
        hopper.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // Typically tied to a whileTrue() binding so it ends when the button is
                      // released.
    }
}
