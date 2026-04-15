// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.JogIntakeArm;
import frc.robot.commands.MoveToPose;
import frc.robot.commands.PassingShot;
import frc.robot.commands.SendItCommand;
import frc.robot.commands.TurnAndZeroCommand;
import frc.robot.utils.AutoFunctions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PassingAuto extends SequentialCommandGroup {
  /** Creates a new TestAutoStuffCommand. */
  public PassingAuto(boolean left) {
    // left and right poses
    Pose2d rightPos1 = new Pose2d(7.5, 1.25, Rotation2d.fromDegrees(45));
    Pose2d rightPos2 = new Pose2d(7.5, 6.50, Rotation2d.fromDegrees(45));
    Pose2d rightPos3 = new Pose2d(8.0, 6.5, Rotation2d.fromDegrees(-45));
    Pose2d rightPos4 = new Pose2d(8.0, 1.25, Rotation2d.fromDegrees(-45));
    Pose2d rightPos5 = new Pose2d(8.5, 1.25, Rotation2d.fromDegrees(45));
    Pose2d rightPos6 = new Pose2d(8.5, 6.50, Rotation2d.fromDegrees(45));

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> RobotContainer.odometry.setPose(AutoFunctions.redVsBlue(new Pose2d()))),
      new SendItCommand(4, 0).withTimeout(1.25),
      new TurnAndZeroCommand(left),
      //new JogIntakeArm(-0.1).withTimeout(0.1),
      new InstantCommand(() -> RobotContainer.intake.intake(), RobotContainer.intake),
      new ParallelCommandGroup(
        new InstantCommand(() -> RobotContainer.uptake.feedShooter(), RobotContainer.uptake),
        new InstantCommand(() -> RobotContainer.hopperFeed.feed(), RobotContainer.hopperFeed),
        new PassingShot(RobotContainer.leftShooter, RobotContainer.rightShooter),
        new SequentialCommandGroup(
          // Move to pos already does the red vs blue for us!
          new MoveToPose(1,2, AutoFunctions.leftVsRight(rightPos1, left)),//.withTimeout(2),
          new MoveToPose(1,2, AutoFunctions.leftVsRight(rightPos2, left)),
          new MoveToPose(1,2, AutoFunctions.leftVsRight(rightPos3, left)),
          new MoveToPose(1,2, AutoFunctions.leftVsRight(rightPos4, left)),
          new MoveToPose(1,2, AutoFunctions.leftVsRight(rightPos5, left)),
          new MoveToPose(1,2, AutoFunctions.leftVsRight(rightPos6, left)),
          //new MoveToPose(1,2, (left?leftPos3:rightPos3)),//.withTimeout(3),
          new InstantCommand(() -> RobotContainer.intake.stop(), RobotContainer.intake)
        )
      )
    );
  }
}
