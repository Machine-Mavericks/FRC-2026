package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;


/** Subsystem */
public class Shooter extends SubsystemBase {

    TalonFX shooterMotor = new TalonFX(1);
    public double wheelRPM;
    public double speed;

    // Local objects and variables here
    // These are for things that only belong to, and used by, the subsystem

    /** Place code here to initialize subsystem */
    public Shooter() {
       speed = 0.0;
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
        shooterMotor.set(speed);
         wheelRPM = shooterMotor.getVelocity().getValueAsDouble()*34.614; // * 60 to RPM * 0.5769 for gear reduction
    }

     public void setShooterSpeed(double power){
        if (Math.abs(power) <= 1){
            speed = power;
        }
        
     }

     public double calculateSpeed(){
        double x = RobotContainer.odometry.getDistanceToGoal();
        double y = (0.000284701*x*x) + 50.67876;
        return y;
     }
    // place special subsystem methods here
    // this is where rest of program can access functions to return
    // values or control the subsystem


}
