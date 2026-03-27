// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnAndZeroCommand extends Command {

  Long timeout = Long.MAX_VALUE;

  /** Creates a new TurnAndZeroCommand. */
  public TurnAndZeroCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drivesystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.limelightShooter.isTargetPresent()) {
      RobotContainer.drivesystem.RobotDrive(0, 0, -0, true);
      if (timeout == Long.MAX_VALUE){
        timeout = System.currentTimeMillis() + 250;
      }
    } else {
      RobotContainer.drivesystem.RobotDrive(0, 0, -1.25, false);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if (!interrupted)
     RobotContainer.odometry.updateAprilTagOdometry(RobotContainer.limelightShooter);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() > timeout;
  }
}
