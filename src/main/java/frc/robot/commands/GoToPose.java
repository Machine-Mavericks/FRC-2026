package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.AutoFunctions;
import frc.robot.utils.Utils;


// command template
public class GoToPose extends Command {


    // target position, speed and rotational speed
    private Pose2d m_target_unmirrored;
    private Pose2d m_target;
    private double m_speed;
    private double m_rotspeed;
  
    // command timeout and counter
    private double m_timeout;
    private Timer m_Timer;

    // x, y, rotation PID controllers to get us to the intended destination
    private PIDController m_xController; 
    private PIDController m_yController;
    private PIDController m_rotController;

    // final position tolerance (m) / angle tolerance (deg) to consider we have arrived at destination
    private final double m_positiontolerance = 0.01;
    private final double m_angletolerance = 1.0;

    // constructor
    public GoToPose(SwerveDrive drivesystem,
                    Pose2d target,
                    double maxspeed,
                    double maxrotationalspeed,
                    double timeout) {

        // this command requires use of swervedrive subsystem
        addRequirements(drivesystem);
    
        // set up PIDs
        m_xController = new PIDController(2.2, 0.1, 0.0);
        m_yController = new PIDController(2.2, 0.1, 0.0);
        m_rotController = new PIDController(0.05, 0.001, 0.0000);
   
        m_xController.setIZone(0.1);
        //m_xController.setIntegratorRange(-5.0, 5.0);
        m_yController.setIZone(0.1);
        //m_yController.setIntegratorRange(-5.0, 5.0);

        // record target
        m_target_unmirrored = target;

        // record speed limits
        m_speed = maxspeed;
        m_rotspeed = maxrotationalspeed;

        // create timer, and record timeout limit
        m_Timer = new Timer();
        m_timeout = timeout;                        
    }

    // This method is called once when command is started
    @Override
    public void initialize() {

        // reset and start in this command
        m_Timer.reset();
        m_Timer.start();

        // reset positional controllers
        m_xController.reset();
        m_yController.reset();
        m_rotController.reset();

        m_target = AutoFunctions.redVsBlue(m_target_unmirrored);
    }

    double xierror = 0.0;

    // This method is called periodically while command is active
    @Override
    public void execute() {

        // get current position estimate from estimator
       // Pose2d currentpose = RobotContainer.odometry.getPose2d();

        // only integrate errors in within 10cm or 5deg of target
        // if (Math.abs(m_target.getX() - currentpose.getX())<0.20)
        //     {m_xController.setI(0.6);m_xController.setP(0.2);}
        // else
        //     {m_xController.setI(0.0);m_xController.setP(2.0);}

        // if (Math.abs(m_target.getY() - currentpose.getY())<0.20)
        //     {m_yController.setI(0.6);m_yController.setP(0.2);}
        // else
        //     {m_yController.setI(0.0);m_yController.setP(2.0);}
        // if (Math.abs(Utils.AngleDifference(m_target.getRotation().getDegrees(),currentpose.getRotation().getDegrees()))<5.0)
        //     m_rotController.setI(0.05);
        // else
        //     m_rotController.setI(0.0); 

        // execute PIDs
        

        // // limit speeds to allowable
        // if (xSpeed > m_speed)
        // xSpeed = m_speed;
        // if (xSpeed < -m_speed)
        // xSpeed = -m_speed;
        // if (ySpeed > m_speed)
        // ySpeed = m_speed; 
        // if (ySpeed < -m_speed)
        // ySpeed = -m_speed;  
        // if (rotSpeed >m_rotspeed)
        // rotSpeed = m_rotspeed;
        // if (rotSpeed < -m_rotspeed)
        // rotSpeed = -m_rotspeed;

        // drive robot according to x,y,rot PID controller speeds
        //RobotContainer.drivesystem.FieldDrive(xSpeed, ySpeed, rotSpeed, false);   

    }

    // This method to return true only when command is to finish. Otherwise return false
   // @Override
   // public boolean isFinished() {

        //Pose2d CurrentPosition = RobotContainer.odometry.getPose2d();

        // we are finished if we are within erorr of target or command had timeed out
        // return (((Math.abs(m_target.getX() - CurrentPosition.getX()) <  m_positiontolerance) &&
        //     (Math.abs(m_target.getY() - CurrentPosition.getY()) <  m_positiontolerance) &&
        //     (Math.abs(m_target.getRotation().getDegrees() - CurrentPosition.getRotation().getDegrees()) < m_angletolerance)) ||
        //     (m_Timer.hasElapsed(m_timeout)));

    //}

    // This method is called once when command is finished.
    // @Override
    // public void end(boolean interrupted) {
    //     // we have finished path. Stop robot
    //     RobotContainer.drivesystem.FieldDrive(0.0, 0.0, 0.0, false);
    // }

}