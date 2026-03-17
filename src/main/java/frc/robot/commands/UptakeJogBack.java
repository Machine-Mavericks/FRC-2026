// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.HopperFeed;
import frc.robot.subsystems.Uptake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class UptakeJogBack extends Command {
  private Uptake uptake;

  /** Creates a new RunIntakeCommand. */
  public UptakeJogBack(Uptake uptake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(uptake);
    this.uptake = uptake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    uptake.jogBack();
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    uptake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
