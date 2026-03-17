// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.HopperJogBack;
import frc.robot.commands.UptakeAndFeed;
import frc.robot.commands.UptakeJogBack;
import frc.robot.subsystems.HopperFeed;
import frc.robot.subsystems.Uptake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShakeAndFeedCommand extends SequentialCommandGroup {
  /** Creates a new FeedCommand. */
  public ShakeAndFeedCommand(HopperFeed hopper, Uptake uptake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new HopperJogBack(hopper).alongWith(new UptakeJogBack(uptake)).withTimeout(0.25),
      new UptakeAndFeed(hopper, uptake)
    );
  }
}
