// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


// These functions are used to mirror points from blue to red
public class AutoFunctions {

    private static final double FieldYSize = 317.7 * 0.0254;
    private static final double FieldXSize = 651.2 * 0.0254; 

    // mirror provided pose2d for red vs blue team
    /**
     * */
    public static Pose2d redVsBlue(Pose2d pose) {
        if (DriverStation.getAlliance().get() == Alliance.Red)
            return new Pose2d (FieldXSize-pose.getX(),
                                FieldYSize-pose.getY(),
                            new Rotation2d(pose.getRotation().getRadians()-Math.PI));
        else
            return pose;
    }

    // mirror provided translation2d for red vs blue team
    public static Translation2d redVsBlue(Translation2d translation) {
        if (DriverStation.getAlliance().get() == Alliance.Red)
            return new Translation2d (FieldXSize-translation.getX(),
                                    FieldYSize-translation.getY());
        else
            return translation;
    }

    // mirror provided rotation2d for red vs blue team
    public static Rotation2d redVsBlue(Rotation2d rotation) {
        if (DriverStation.getAlliance().get() == Alliance.Red)
            return new Rotation2d (rotation.getRadians() - Math.PI);
        else
            return rotation;
    }

    /**
     * Get a pose mirrored left vs right
     * @param pose Pose on the right side of the field
     * @param left Boolean wehther to return right or left version
     * @return Right or left version of provided pose
     */
    public static Pose2d leftVsRight(Pose2d pose, boolean left){
        if (left) {
            return  new Pose2d(pose.getX(), 8.5 - pose.getY(), Rotation2d.fromDegrees(360).minus(pose.getRotation()));
        } else {
            return pose;
        }
    }

}
