package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.utils.AutoFunctions;
import frc.robot.utils.Utils;

public class MoveToPose extends Command {

    private Pose2d dest_unmirrored;
    private Pose2d dest;
    private double maxSpeed;
    private double maxRotSpeed;
    private boolean redVsBlueEnable;

    private double m_timeout;
    private Timer m_Timer;

    private PIDController m_xController;
    private PIDController m_yController;
    private PIDController m_rotController;

    private final double m_positiontolerance = 0.1;
    private final double m_angletolerance = 3.0;

     private StructPublisher<Pose2d> posePublisher;

    public MoveToPose(double maxSpeed, double maxAccel, Pose2d destination) {
        this(maxSpeed, maxAccel, destination, true);
    }

    public MoveToPose(double maxSpeed,
                    double maxAccel,
                    Pose2d destination,
                    boolean redVsBlueEnable) {

        // record input parameters
        this.maxSpeed = maxSpeed;
        this.maxRotSpeed = maxAccel; // Assuming maxAccel is maxRotSpeed
        this.dest_unmirrored = destination;
        this.redVsBlueEnable = redVsBlueEnable;
        this.m_timeout = 15.0; // set default timeout to 15 seconds

        // this command requires robot drive subsystem
        addRequirements(RobotContainer.drivesystem);

        // set up PIDs
        m_xController = new PIDController(3, 0.1, 0.0);
        m_yController = new PIDController(3, 0.1, 0.0);
        m_rotController = new PIDController(0.05, 0.001, 0.0000);
   
        m_xController.setIZone(0.1);
        m_yController.setIZone(0.1);

        m_rotController.enableContinuousInput(-180, 180);

        // create timer
        m_Timer = new Timer();
    }

    // This method is called once when command is started
    @Override
    public void initialize() {
        // reset and start timer
        m_Timer.reset();
        m_Timer.start();

        // reset positional controllers
        m_xController.reset();
        m_yController.reset();
        m_rotController.reset();

        // destination mirroring as req'd
        if (redVsBlueEnable)
            dest = AutoFunctions.redVsBlue(dest_unmirrored);
        else
            dest = dest_unmirrored;

        posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic( "MoveToPose/robotPose", Pose2d.struct).publish();
    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
        // get current position estimate from estimator
        Pose2d currentpose = RobotContainer.odometry.getPose2d();

        // execute PIDs
        double xSpeed = m_xController.calculate(currentpose.getX(), dest.getX());
        double ySpeed = m_yController.calculate(currentpose.getY(), dest.getY());
        double rotSpeed = m_rotController.calculate(currentpose.getRotation().getDegrees(), dest.getRotation().getDegrees());

        SmartDashboard.putNumber("Autos/X Speed", xSpeed);
        SmartDashboard.putNumber("Autos/Y Speed", ySpeed);
        SmartDashboard.putNumber("Autos/Rot Speed", rotSpeed);

        Translation2d error = dest.getTranslation().minus(currentpose.getTranslation());
        SmartDashboard.putNumber("Autos/X Error", Math.abs(dest.getX() - currentpose.getX()));
        SmartDashboard.putNumber("Autos/Y Error", Math.abs(dest.getY() - currentpose.getY()));
        SmartDashboard.putNumber("Autos/Rot Error", Math.abs(Utils.AngleDifference(dest.getRotation().getDegrees(), currentpose.getRotation().getDegrees())));

        SmartDashboard.putNumber("Autos/Current Rotation", currentpose.getRotation().getDegrees());
        
        posePublisher.set(dest);

        // limit speeds to allowable
        if (xSpeed > maxSpeed) xSpeed = maxSpeed;
        if (xSpeed < -maxSpeed) xSpeed = -maxSpeed;
        if (ySpeed > maxSpeed) ySpeed = maxSpeed; 
        if (ySpeed < -maxSpeed) ySpeed = -maxSpeed;  
        if (rotSpeed > maxRotSpeed) rotSpeed = maxRotSpeed;
        if (rotSpeed < -maxRotSpeed) rotSpeed = -maxRotSpeed;

        // drive robot according to x,y,rot PID controller speeds
        RobotContainer.drivesystem.FieldDrive(xSpeed, ySpeed, rotSpeed, false);   
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {
        Pose2d CurrentPosition = RobotContainer.odometry.getPose2d();

        // we are finished if we are within erorr of target or command had timeed out
        return ((Math.abs(dest.getX() - CurrentPosition.getX()) <  m_positiontolerance) &&
            (Math.abs(dest.getY() - CurrentPosition.getY()) <  m_positiontolerance) &&
            (Math.abs(Utils.AngleDifference(dest.getRotation().getDegrees(), CurrentPosition.getRotation().getDegrees())) < m_angletolerance)) ||
            (m_Timer.hasElapsed(m_timeout));
    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
        // we have finished path. Stop robot
        RobotContainer.drivesystem.FieldDrive(0.0, 0.0, 0.0, false);
    }
}