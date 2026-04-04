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
import frc.robot.commands.MoveToPose;
import frc.robot.commands.SendItCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TurnAndZeroCommand;
import frc.robot.commands.UptakeAndFeed;
import frc.robot.utils.AutoFunctions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StraightAuto extends SequentialCommandGroup {
  /** Creates a new TestAutoStuffCommand. */
  public StraightAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SendItCommand(4, 0).withTimeout(1.25),
      new TurnAndZeroCommand(false),
      new InstantCommand(() -> RobotContainer.intake.intake(), RobotContainer.intake),
      // Move to pos already does the red vs blue for us!
      new MoveToPose(1,1, (new Pose2d(8.25, 3.0, new Rotation2d()))).withTimeout(5),
      new MoveToPose(3,3, (new Pose2d(5.5, 2.64, Rotation2d.fromDegrees(180)))).withTimeout(3),
      new InstantCommand(() -> RobotContainer.intake.stop(), RobotContainer.intake),
      new SendItCommand(4, 0).withTimeout(1.5),
      
      new ParallelCommandGroup(
        new ShootCommand(RobotContainer.leftShooter, RobotContainer.rightShooter),
        
        new SequentialCommandGroup(
          new TurnAndZeroCommand(true),
          new MoveToPose(1,1, (new Pose2d(2.0, 2.64, Rotation2d.fromDegrees(225)))).withTimeout(2.5),
          new UptakeAndFeed(RobotContainer.hopperFeed, RobotContainer.uptake)
        )
      )


      //new MoveToPose(1, 1, new Pose2d(1, 1, new Rotation2d()))
    );
  }
}
