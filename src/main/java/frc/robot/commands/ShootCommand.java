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
import frc.robot.utils.ShooterCalculations;
import frc.robot.utils.TargetCalculations;

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
    Pose2d robotPose = RobotContainer.odometry.getPose2d();

    Pose2d targetPose = TargetCalculations.getTargetPose(robotPose);

    Pose2d leftTurretPose = RobotContainer.turretLeft.getFieldPose();
    double leftTurretDistance = distanceBetweenPoses(targetPose, leftTurretPose);
    double leftSpeedRPS = RobotContainer.leftShooter.CalculateSpeed(leftTurretDistance);
    leftShooter.shooterSpeed(leftSpeedRPS);

    Pose2d rightTurretPose = RobotContainer.turretRight.getFieldPose();
    double rightTurretDistance = distanceBetweenPoses(targetPose, rightTurretPose);
    double rightSpeedRPS = RobotContainer.rightShooter.CalculateSpeed(rightTurretDistance);
    rightShooter.shooterSpeed(rightSpeedRPS);

    SmartDashboard.putNumber("ShootCommand/Left Distance", leftTurretDistance);
    SmartDashboard.putNumber("ShootCommand/Right Distance", rightTurretDistance);
    SmartDashboard.putNumber("ShootCommand/LeftSpeedRPS", leftSpeedRPS);
    SmartDashboard.putNumber("ShootCommand/RightSpeedRPS", rightSpeedRPS);
  }

  private double distanceBetweenPoses(Pose2d a, Pose2d b){
    // Use inner translation objects to get distance
    return a.getTranslation().getDistance(b.getTranslation());
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
