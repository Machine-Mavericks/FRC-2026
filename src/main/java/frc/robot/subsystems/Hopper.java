package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/** Subsystem */
public class Hopper extends SubsystemBase {
    
    TalonFX hopperMotor = new TalonFX(5);
    
    @Logged
    private double commandedSpeed;
    public double currentSpeed;

    // Local objects and variables here
    // These are for things that only belong to, and used by, the subsystem

    /** Place code here to initialize subsystem */
    public Hopper() {

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
   public void periodic() {
        hopperMotor.set(commandedSpeed);
        currentSpeed = hopperMotor.get();
    }

    public void hopperDown(){
      
    }

    // place special subsystem methods here
    // this is where rest of program can access functions to return
    // values or control the subsystem


}
