// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.AutoFunctions;
import frc.robot.utils.Utils;

// command controls mecanum in manual mode
public class ManualDrive extends Command {

    // PID controller used to counteract rotational drift due to misalignment of wheels
    // note: FRC 2024 used P=0.01, I=0.0, D=0.0
    // FTC 2024 used P=0.07, I=0.0005, D=0.0
    private PIDController m_headingPID = new PIDController(0.08, 0.2, 0);  // p=0.04 i=0.0005
    private Double m_PIDTarget = null;    // Use Double class so it can be set to null
    private long m_pidDelay = -1;

    

    double powerFactor;
    double basePowerFacter = 0.12;
    double boostPowerFacter = 0.88;

    final double MAX_ACCEL = 100.0;  // max accel in m/s2

    double old_dX, old_dY;

    Timer deltat;

    // rotate to reef controller
    PIDController omegaControl;

    SwerveDrive drivesystem;
    CommandXboxController driverOp;
    Pigeon gyro;

    // constructor
    public ManualDrive(SwerveDrive drivesystem, Pigeon gyro, CommandXboxController driverOp) {

        // this command requires swerve drive subsystem
        this.drivesystem = drivesystem;
        this.driverOp = driverOp;
        this.gyro = gyro;
        addRequirements(drivesystem);

        deltat = new Timer();

        omegaControl = new PIDController(0.09, 0.001, 0.0000);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {
        m_headingPID.setIntegratorRange(-7.0, 7.0);

        
        
        m_PIDTarget = null;
        m_pidDelay = 10;
        old_dX=0.0;
        old_dY = 0.0;
        deltat.reset();
        deltat.start();
        omegaControl.reset();

    }

    // This method is called periodically while command is active
    @Override
    public void execute() {

        // get joystick input - for competition
        // Note: FRC joystick is oriented +ve to left and +ve down
        double dX = -driverOp.getLeftY();
        double dY = -driverOp.getLeftX();
        double omega = -2.0 * driverOp.getRightX();
        double speedTrigger = driverOp.getRightTriggerAxis();
        
        //boolean Park = RobotContainer.driverOp.leftBumper().getAsBoolean();

        // if on red team, reverse x and y movements to match FRC field coordinates
        if (DriverStation.getAlliance().get() == Alliance.Red)
        {
            dX = dX * -1;
            dY = dY * -1;
        }


        // implement dead-zoning of joystick inputs
        // made the dead zones a circle instead of a square, old equation commented below
        dX = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2)) > 0.1 ? dX : 0;
        dY = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2)) > 0.1 ? dY : 0;
        omega = Math.sqrt(Math.pow(omega, 2) + Math.pow(driverOp.getRightY(), 2)) > 0.1  ? omega : 0;
        // dX = Math.abs(dX) > 0.1 ? dX : 0;
        // dY = Math.abs(dY) > 0.1 ? dY : 0;
        // omega = Math.abs(omega) > 0.2 ? omega : 0;
        
        // determine distance from center of reef to robot
        //Translation2d CurrentPosition = RobotContainer.odometry.getPose2d().getTranslation();
        Translation2d CenterReef = AutoFunctions.redVsBlue(new Translation2d(4.489, 4.0259));
        

        boolean EnableDriftCorrection = true;
        
        
            
            // determine distance from center of reef
           

            // heading to center of reef
        
            
            // current robot angle
            double RobotAngle = gyro.getYawAngle();

          

        powerFactor = basePowerFacter + (speedTrigger * boostPowerFacter);
        // Since the drive was shifted to closed loop (i.e. requested velocities), change joystick input max values
        // to MAX_SPEED values.
        powerFactor = powerFactor * drivesystem.MAX_SPEED;

        // include power factor to get full x,y and omega speeds beings requested
        dX *= powerFactor;
        dY *= powerFactor;
        omega *= powerFactor;

        // limit acceleration of robot to MAX_ACCEL - to reduce chance of wheel-slide
        double t = deltat.get();
        if ((dX > old_dX + t*MAX_ACCEL) && (old_dX>0))
            dX = old_dX + t*MAX_ACCEL;
        if ((dX <  old_dX - t*MAX_ACCEL) && (old_dX<0))
            dX = old_dX - t*MAX_ACCEL;

        if ((dY > old_dY + t*MAX_ACCEL) && (old_dY>0))
            dY = old_dY + t*MAX_ACCEL;
        if ((dY < old_dY - t*MAX_ACCEL) && (old_dY<0))
            dY = old_dY - t*MAX_ACCEL;

        // save speeds for use next time
        old_dX = dX;
        old_dY = dY;
        deltat.reset();

        // drive robot
        //if (!Park)
            drivesystem.FieldDrive(dX, dY, omega, false);
        //else
            //RobotContainer.drivesystem.FieldDrive(0.0, 0.0, 0.0, true);
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        return false;
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {

    }

}