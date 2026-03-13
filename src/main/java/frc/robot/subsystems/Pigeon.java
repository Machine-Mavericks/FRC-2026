// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.RobotMap;


public class Pigeon extends SubsystemBase {

    //gyro offset adjust
    private double OffsetAdjust;

    // make our gyro object
    private static Pigeon2 gyro;

    /** Creates a new Gyro. */
    public Pigeon() {
        // initialize shuffleboard
        initializeShuffleboard();
    
        // make pigeon object
        gyro = new Pigeon2(RobotMap.CANID.PIGEON);
        gyro.reset();

        // offset adjust
        OffsetAdjust = 0.0;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateShuffleboard();
    }

    /** Gets the yaw of the robot
    * @return current yaw value (-180 to 180) */
    public double getYawAngle() {
    
        // scaling factor for CTR Pigeon determine by test - Feb 5 2023
        // in 2023, best scaling factor was 0.99895833
        double value = gyro.getYaw().getValueAsDouble()*1.0 + OffsetAdjust;
    
        // convert continous number to -180 to +180deg to match NavX function call
        if (value > 0)
            return ((value+180.0) %360.0)-180.0;
        else
            return ((value-180.0) %360.0)+180.0;
    }

  
    /** Resets yaw to zero  */
    public void resetYawAngle() {
    
        setYawAngle(0.0);
    } 
  
    /** Resets yaw to a value */
    public void setYawAngle(double angle) {
    
        // reset our Gyro
        OffsetAdjust -= getYawAngle() - angle;
    }
  

    /** Gets the pitch of the robot
    * @return current pitch value in deg */
    public double getPitchAngle() {
        return gyro.getPitch().getValueAsDouble();
    }

    /** Get Roll
    * @return roll in deg */
    public double getRollAngle() {
        return gyro.getRoll().getValueAsDouble();
    }

    /** X Acceleration
    * @return ratio of gravity */
    public double getXAcceleration() {
        return gyro.getAccelerationX().getValueAsDouble();
    }

    /** Y Acceleration
    * @return ratio of gravity */
    public double getYAcceleration() {
        return gyro.getAccelerationY().getValueAsDouble();
    }

    
    /** Gyro Shuffleboard */

  
    // -------------------- Subsystem Shuffleboard Methods --------------------

    // subsystem shuffleboard controls
    private GenericEntry m_gyroPitch;
    private GenericEntry m_gyroYaw;
    private GenericEntry m_gyroRoll;
    private GenericEntry m_xAcceleration;
    private GenericEntry m_yAcceleration;


    /** Initialize subsystem shuffleboard page and controls */
    private void initializeShuffleboard() {
        // Create odometry page in shuffleboard
        ShuffleboardTab Tab = Shuffleboard.getTab("Pigeon");

        // create controls to display robot position, angle, and gyro angle
        ShuffleboardLayout l1 = Tab.getLayout("Pigeon Values", BuiltInLayouts.kList);
        l1.withPosition(0, 0);
        l1.withSize(2, 4);
        m_gyroPitch = l1.add("Pitch (deg)", 0.0).getEntry();
        m_gyroYaw = l1.add("Yaw (deg)", 0.0).getEntry();
        m_gyroRoll = l1.add("Roll (deg)", 0.0).getEntry();
        m_xAcceleration = l1.add("X Acceleration", 0.0).getEntry();
        m_yAcceleration = l1.add("Y Acceleration", 0.0).getEntry();

    }

    /** Update subsystem shuffle board page with current Gyro values */
    private void updateShuffleboard() {
        // write current robot Gyro
        m_gyroPitch.setDouble(getPitchAngle());
        m_gyroYaw.setDouble(getYawAngle());
        m_gyroRoll.setDouble(getRollAngle());
        m_xAcceleration.setDouble(getXAcceleration());
        m_yAcceleration.setDouble(getYAcceleration());
    }

}
