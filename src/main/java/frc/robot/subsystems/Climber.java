package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


/** Subsystem */
public class Climber extends SubsystemBase {

    TalonFX climbMotor = new TalonFX(5);
    // Local objects and variables here
    // These are for things that only belong to, and used by, the subsystem

    /** Place code here to initialize subsystem */
    public Climber() {

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }

    public void setClimbPosition(double position){
        climbMotor.setPosition(position);
    }
    // place special subsystem methods here
    // this is where rest of program can access functions to return
    // values or control the subsystem


}
