package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;



// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup
public class IntakeCommand extends SequentialCommandGroup {
   
    // constructor
    public IntakeCommand() {

        addCommands (
        
        new InstantCommand(()-> RobotContainer.hopper.hopperDown()),

        new InstantCommand(()-> RobotContainer.intake.intakeRun(0.5))
        

       
        );
    }

}


