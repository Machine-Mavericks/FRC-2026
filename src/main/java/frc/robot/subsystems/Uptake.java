package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;


/** Subsystem */
public class Uptake extends SubsystemBase {
    
 TalonFX uptakeMotor = new TalonFX(RobotMap.CANID.UPTAKE_MOTOR);
 
@Logged
 private double commandedSpeed;
 public double currentSpeed;

 private static final double MECHANISM_RATIO = (1.0 / 9.0);
 
    // Local objects and variables here
    // These are for things that only belong to, and used by, the subsystem

    /** Place code here to initialize subsystem */
    public Uptake() { 

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
        uptakeMotor.set(commandedSpeed);
        currentSpeed = uptakeMotor.get();
    }

    public void spinUptake(double speed){
        commandedSpeed = speed;
    }

    // place special subsystem methods here
    // this is where rest of program can access functions to return
    // values or control the subsystem

    public void uptakeRun (){
        spinUptake(1.0); 
    }

    public void uptakeStop(){
        spinUptake(0.0);  
    }

    @Logged(name = "Uptake Current Speed")
    public double getCurrentSpeed(){
        return currentSpeed; 
    }

    @Logged(name = "Uptake Commanded Speed")
    public double getCommandedSpeed(){
        return commandedSpeed;
    }


}
