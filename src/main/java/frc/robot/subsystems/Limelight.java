// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LimelightHelpers.RawTarget;


public class Limelight extends SubsystemBase {

    public final String name;

    private StructPublisher<Pose2d> posePublisher;
    private IntegerPublisher tagPublisher;

    public Limelight(String name) {
        this.name = "limelight-" + name;
        posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic(this.name + "/robotPose", Pose2d.struct).publish();
        tagPublisher = NetworkTableInstance.getDefault().getIntegerTopic(this.name + "/visible tags").publish();

    }

    public void setCameraPoseRobotSpace(double[] position) {
        // TODO: Implement
    }

    public void SetFiducialIDFiltersOverride(int[] tags){
        // TODO: Imlement
    }

    public boolean isTargetPresent () {
        return LimelightHelpers.getTargetCount(name) > 0;
    }


    @Override
    public void periodic(){

        double yaw = RobotContainer.gyro.getYawAngle();

        LimelightHelpers.SetRobotOrientation(name, yaw, 0.0, 0.0, 0.0, 0.0, 0.0);

        posePublisher.set(getPose());
        tagPublisher.set(LimelightHelpers.getTargetCount(name));
    }

    public Pose2d getPose(){
        LimelightHelpers.PoseEstimate measurement;
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue){
            measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        } else {
            measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);      
        }
        return measurement.pose;
    }


}
    