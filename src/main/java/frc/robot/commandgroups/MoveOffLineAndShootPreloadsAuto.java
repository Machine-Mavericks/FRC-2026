package frc.robot.commandgroups;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.RobotMap.HopperFeed;
import frc.robot.commands.MoveToPose;
import frc.robot.commands.Pause;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.UptakeAndFeed;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.AutoFunctions;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup
public class MoveOffLineAndShootPreloadsAuto extends SequentialCommandGroup {

    
    // constructor
    public MoveOffLineAndShootPreloadsAuto() {

        addCommands (
            
        // depending on start position, set odometry
        // depending on start position, set odometry
        new InstantCommand(()-> {
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
                    startpose = new Pose2d(3.42, 0.63, new Rotation2d(Math.toRadians(60.0))); // trench right
                    break;
                case 3:
                    startpose = new Pose2d(3.42, 7.42, new Rotation2d(Math.toRadians(300.0))); // trench left
                    break;          
                default:
                    startpose = new Pose2d(3.42, 5.72, new Rotation2d(Math.toRadians(315.0)));
            };

            // convert for red vs blue
            startpose = AutoFunctions.redVsBlue(startpose);
            
            // set odometry
            RobotContainer.odometry.setPose(startpose);

        
        } ),
        
        // Turn on tag detection
        new InstantCommand(()->RobotContainer.odometry.TagEnable=true),

        // Move off the line 
        // new MoveToPose(2.0, 
        //                 2.5,
        //                 new Pose2d (3.42,2.42, new Rotation2d(Math.toRadians()))
        //),

        // Spinup Shoot
        new ShootCommand(RobotContainer.leftShooter, RobotContainer.rightShooter),

        // Jog back hopper to clear anything thats stuck 
        new InstantCommand(()-> RobotContainer.hopperFeed.jogBack()),
        
        // How long the jog back is 
        new Pause(1.0),

        // Feed hopper
        new InstantCommand(()-> RobotContainer.hopperFeed.feed()),

        // How long the feed is 
        new Pause(1.0),

        // Run Uptake 
        new InstantCommand(()-> RobotContainer.uptake.feedShooter()),

        // How long uptake will run
        new Pause(1.0),

         // Turn on tag detection
        new InstantCommand(()->RobotContainer.odometry.TagEnable=false)

        

        
        // Turn on tag detection
        //new InstantCommand(()->RobotContainer.odometry.TagEnable=true),

        //new Pause(1.0)

    
        // move slowly to exact spot               
        //new MoveToPose(0.5, 
        //               1.0,
        //               new Pose2d (5.40,2.75, new Rotation2d(Math.toRadians(120)))),

        
        // Turn off tag detection
        //new InstantCommand(()->RobotContainer.odometry.TagEnable=false)
        
        

       
        );
    }

}


