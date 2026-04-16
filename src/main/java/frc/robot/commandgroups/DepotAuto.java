// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import java.time.Instant;

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
public class DepotAuto extends SequentialCommandGroup {
  /** Creates a new TestAutoStuffCommand. */
  public DepotAuto(boolean isSubcommand) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> {
        if (!isSubcommand)
          RobotContainer.odometry.setPose(AutoFunctions.redVsBlue(new Pose2d(3.62 ,4, Rotation2d.fromDegrees(180))));
      }, RobotContainer.odometry),

       new InstantCommand(() -> RobotContainer.intake.intake(), RobotContainer.intake),
      // Move to pos already does the red vs blue for us!
      new MoveToPose(1,1, (new Pose2d(1.27, 6.0, Rotation2d.fromDegrees(180)))),

      new MoveToPose(1,1, (new Pose2d(0.27, 6, Rotation2d.fromDegrees(180)))),
      
      new MoveToPose(1,1, (new Pose2d(1.75, 5, Rotation2d.fromDegrees(180)))),

      new MoveToPose(1,1, (new Pose2d(1.75, 4.0, Rotation2d.fromDegrees(180)))),

      new InstantCommand(() -> RobotContainer.odometry.updateAprilTagOdometry(RobotContainer.limelightShooter)),

      new InstantCommand(() -> RobotContainer.intake.stop(), RobotContainer.intake),
      //new SendItCommand(4, 0).withTimeout(1.5),
      
      new ParallelCommandGroup(
        new ShootCommand(RobotContainer.leftShooter, RobotContainer.rightShooter),
        new UptakeAndFeed(RobotContainer.hopperFeed, RobotContainer.uptake)
      )


      //new MoveToPose(1, 1, new Pose2d(1, 1, new Rotation2d()))
    );
  }
}
