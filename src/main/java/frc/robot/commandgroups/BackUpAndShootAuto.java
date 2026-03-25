package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.commands.MoveToPose;
import frc.robot.commands.Pause;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.UptakeAndFeed;
import frc.robot.utils.AutoFunctions;

/**
 * 
 * 
 * All target coordinates are in Blue Alliance coordinates!!!
 * We transform to RED usind AutoFunctions.redVsBlue()
 * 
 * 
 * 
 */
public class BackUpAndShootAuto extends SequentialCommandGroup {

    public BackUpAndShootAuto() {
        addRequirements(RobotContainer.drivesystem);

        addCommands(
                // 1. Set odometry to start pose from Shuffleboard
                new InstantCommand(() -> {
                    int startposn = RobotContainer.mainShufflePage.getStartPositionIndex();
                    Pose2d startpose;
                    switch (startposn) {
                        case 0: // Right Bump
                            startpose = new Pose2d(3.42, 2.33, new Rotation2d(Math.toRadians(45.0)));
                            break;
                        case 1: // Left Bump
                            startpose = new Pose2d(3.42, 5.72, new Rotation2d(Math.toRadians(315.0)));
                            break;
                        case 2: // Trench Right
                            startpose = new Pose2d(3.42, 0.63, new Rotation2d(Math.toRadians(180.0)));
                            break;
                        case 3: // Trench Left
                            startpose = new Pose2d(3.42, 7.42, new Rotation2d(Math.toRadians(300.0)));
                            break;
                        default:
                            startpose = new Pose2d(3.42, 5.72, new Rotation2d(Math.toRadians(315.0)));
                    }
                    ;
                    // Mirror for alliance natively
                    startpose = AutoFunctions.redVsBlue(startpose);
                    RobotContainer.odometry.setPose(startpose);
                }),

                // 2. Enable Tag Detection
                new InstantCommand(() -> RobotContainer.odometry.TagEnable = true),

                // 3. Move to depot while intaking fuel
                new ParallelDeadlineGroup(
                        // Deadline: finish driving to depot
                        new MoveToPose(1.5, 2.0, new Pose2d(1.30, 5.72, new Rotation2d(Math.toRadians(315.0)))),
                        // Continuously run intake while driving
                        new RunCommand(() -> RobotContainer.intake.intake(RobotMap.Intake.INTAKE_DEPOT_SPEED), RobotContainer.intake)),

                // 4. Stop intake once arrived at depot
                new InstantCommand(() -> RobotContainer.intake.stop()),

                // 5. Short pause to settle
                new Pause(0.5),

                // 6. Move to midpoint and face hub
                new MoveToPose(2.0, 2.5, new Pose2d(2.36, 5.72, new Rotation2d(Math.toRadians(323.0)))),

                // 7. Short pause to let turret settle
                new Pause(0.5),

                // 8. Spin up and fire
                new ParallelCommandGroup(
                        new ShootCommand(RobotContainer.leftShooter, RobotContainer.rightShooter),
                        new SequentialCommandGroup(
                                // Wait for both shooters to reach target RPS, max timeout 3s
                                new WaitUntilCommand(() -> RobotContainer.leftShooter.isAtSpeed() &&
                                        RobotContainer.rightShooter.isAtSpeed()).withTimeout(3.0),

                                // Push fuel in
                                new UptakeAndFeed(RobotContainer.hopperFeed, RobotContainer.uptake),

                                // Keep running feed
                                new Pause(6.0))),

                // 9. Disable Tag Detection at end
                new InstantCommand(() -> RobotContainer.odometry.TagEnable = false));
    }
}
