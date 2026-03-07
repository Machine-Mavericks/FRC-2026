package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Automates the process of intaking a ball:
 *   - Press button: arm deploys (down) and intake rollers spin.
 *   - Press button again: intake rollers stop and arm stows (up).
 *
 * Bound with toggleOnTrue() in RobotContainer so the command runs until
 * canceled by a second button press.
 */
public class IntakeSequence extends Command {

    private final IntakeSubsystem intake;
    private final IntakeArm intakeArm;

    /**
     * Creates a new IntakeSequence.
     *
     * @param intake    The intake subsystem to run.
     * @param intakeArm The intake arm subsystem to run.
     */
    public IntakeSequence(IntakeSubsystem intake, IntakeArm intakeArm) {
        this.intake = intake;
        this.intakeArm = intakeArm;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake, intakeArm);
    }

    // Deploy arm and start intake rollers when the command is first scheduled.
    @Override
    public void initialize() {
        intakeArm.moveTo(frc.robot.RobotMap.IntakeArm.DEPLOYED_POSITION);
        intake.intake();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Could stop rollers here if a beam-break/limit switch signals ball captured.
    }

    // Stop rollers and stow arm when canceled (second button press).
    @Override
    public void end(boolean interrupted) {
        intake.stop();
        // Return arm to stowed position via Motion Magic; do not call stop() on arm
        // here so the position hold remains active until the arm reaches home.
        intakeArm.moveTo(frc.robot.RobotMap.IntakeArm.STOWED_POSITION);
    }

    // Command runs indefinitely until toggled off.
    @Override
    public boolean isFinished() {
        return false;
    }
}
