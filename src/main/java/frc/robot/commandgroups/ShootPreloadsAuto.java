package frc.robot.commandgroups;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.RobotMap.HopperFeed;
import frc.robot.commands.MoveToPose;
import frc.robot.commands.Pause;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.SendItCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.UptakeAndFeed;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Uptake;
import frc.robot.utils.AutoFunctions;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup
public class ShootPreloadsAuto extends SequentialCommandGroup {

    // constructor
    public ShootPreloadsAuto(boolean direction) {
        addRequirements(RobotContainer.drivesystem);
        addCommands(

                // depending on start position, set odometry
                new InstantCommand(() -> {
                    int startposn = RobotContainer.mainShufflePage.getStartPositionIndex();

                    Pose2d startpose; // add positions
                    switch (startposn) {
                        case 0: // on the center line 3.4m
                            startpose = new Pose2d(3.42, 2.33, new Rotation2d(Math.toRadians(45.0))); // Bump right
                            break;
                        case 1:
                            startpose = new Pose2d(3.42, 5.72, new Rotation2d(Math.toRadians(315.0))); // Bump left
                            break;
                        case 2:
                            startpose = new Pose2d(3.42, 0.63, new Rotation2d(Math.toRadians(180))); // trench right 60 degrees
                            break;
                        case 3:
                            startpose = new Pose2d(3.42, 7.42, new Rotation2d(Math.toRadians(300.0))); // trench left
                            break;
                        default:
                            startpose = new Pose2d(3.42, 5.72, new Rotation2d(Math.toRadians(315.0)));
                    }
                    ;

                    // convert for red vs blue
                    startpose = AutoFunctions.redVsBlue(startpose);

                    // set odometry
                    RobotContainer.odometry.setPose(startpose);

                }),
                
                // new MoveToPose(
                //     2.0, 
                //     2.5,
                //     AutoFunctions.redVsBlue(new Pose2d (3.92,0.63, new Rotation2d(Math.toRadians(180))))
                // )

                // new InstantCommand(() -> RobotContainer.drivesystem.FieldDrive(1, 0, 0, false)),
                // new Pause(2),
                // new InstantCommand(() -> RobotContainer.drivesystem.FieldDrive(0, 1, 0, false)),
                // new Pause(5),
                // new InstantCommand(() -> RobotContainer.drivesystem.FieldDrive(0,0,0, false))

                // new Pause(10),
                // new SendItCommand(2, 0).withTimeout(1.5),
                // new SendItCommand(0, direction ? 2 : -2).withTimeout(1.75)
                // new MoveToPose(
                //     2.0, 
                //     2.5,
                //     new Pose2d (3.42,2.42, new Rotation2d(Math.toRadians(0)))
                // )
                new ParallelCommandGroup(
                        new ShootCommand(RobotContainer.leftShooter, RobotContainer.rightShooter),

                        new SequentialCommandGroup(
                                // Jog back hopper to clear anything thats stuck
                                //new InstantCommand(() -> RobotContainer.hopperFeed.jogBack()),

                                // How long the jog back is
                                //new Pause(3.0),

                                // Feed hopper
                                // new InstantCommand(() -> RobotContainer.hopperFeed.feed()),

                                // How long the feed is
                                new Pause(1.0),

                                // Run Uptake
                                new UptakeAndFeed(RobotContainer.hopperFeed, RobotContainer.uptake),

                                // How long uptake will run
                                new Pause(20.0))                    
                        
                )
        );
    }

}
