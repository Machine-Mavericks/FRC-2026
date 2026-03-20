// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.AutoFunctions;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {

  private Shooter leftShooter;
  private Shooter rightShooter;

  /** Creates a new ShootCommand. */
  public ShootCommand(Shooter leftShooter, Shooter rightShooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(leftShooter, rightShooter);

    this.leftShooter = leftShooter;
    this.rightShooter = rightShooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = RobotContainer.limelightShooter.getPose();
    Pose2d hubPose = AutoFunctions.redVsBlue(new Pose2d(4.625, 4.034, Rotation2d.kZero));

    double distance = Math.sqrt(Math.pow(robotPose.getX() - hubPose.getX(), 2) + Math.pow(robotPose.getY() - hubPose.getY(), 2));

    double leftSpeedRPS = RobotContainer.leftShooter.CalculateSpeed(distance);
    leftShooter.shooterSpeed(leftSpeedRPS);

    double rightSpeedRPS = RobotContainer.rightShooter.CalculateSpeed(distance);
    rightShooter.shooterSpeed(rightSpeedRPS);

    SmartDashboard.putNumber("ShootCommand/Distance", distance);
    SmartDashboard.putNumber("ShootCommand/LeftSpeedRPS", leftSpeedRPS);
    SmartDashboard.putNumber("ShootCommand/RightSpeedRPS", rightSpeedRPS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leftShooter.shooterSpeed(0);
    rightShooter.shooterSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
