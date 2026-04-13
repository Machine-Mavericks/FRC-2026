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
import frc.robot.commands.SendItCommand;
import frc.robot.commands.TurnAndZeroCommand;
import frc.robot.utils.AutoFunctions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SweepAuto extends SequentialCommandGroup {
  /** Creates a new TestAutoStuffCommand. */
  public SweepAuto(boolean left) {
    // left and right poses
    Pose2d rightPos1 = new Pose2d(7.62, 1.778, new Rotation2d());
    Pose2d leftPos1 = new Pose2d(7.62, 6.274, new Rotation2d());
    Pose2d rightPos2 = new Pose2d(7.62, 4.0, Rotation2d.fromDegrees(90));
    Pose2d leftPos2 = new Pose2d(7.62, 4.05, Rotation2d.fromDegrees(270));
    Pose2d rightPos3 = new Pose2d(5.5, 2.64, Rotation2d.fromDegrees(180));
    Pose2d leftPos3 = new Pose2d(5.5, 5.41, Rotation2d.fromDegrees(180));
    Pose2d rightPos4 = new Pose2d(2.0, 2.64, Rotation2d.fromDegrees(225));
    Pose2d leftPos4 = new Pose2d(2.0, 5.41, Rotation2d.fromDegrees(135));
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SendItCommand(4, 0).withTimeout(1.25),
      new TurnAndZeroCommand(left),
      new JogIntakeArm(-0.1).withTimeout(0.1),
      new InstantCommand(() -> RobotContainer.intake.intake(), RobotContainer.intake),
      // Move to pos already does the red vs blue for us!
      new MoveToPose(1,1, (left?leftPos1:rightPos1)).withTimeout(2),
      new MoveToPose(1,1, (left?leftPos2:rightPos2)).withTimeout(2.5),
      new MoveToPose(1,1, (left?leftPos3:rightPos3)).withTimeout(3),
      new InstantCommand(() -> RobotContainer.intake.stop(), RobotContainer.intake),
      new SendItCommand(4, 0).withTimeout(1.5),
      new TurnAndZeroCommand(!left),
      
      new ParallelCommandGroup(
        new MoveToPose(1,1, (left?leftPos4:rightPos4)).withTimeout(2.5),
        new ShootPreloadsAuto(false)        
      )


      //new MoveToPose(1, 1, new Pose2d(1, 1, new Rotation2d()))
    );
  }
}
