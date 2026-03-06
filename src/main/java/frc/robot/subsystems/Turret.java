package frc.robot.subsystems;

import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


/** Subsystem */
public class Turret extends SubsystemBase {

    SparkMax IntakeMotor = new SparkMax(3, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    // Local objects and variables here
    // These are for things that only belong to, and used by, the subsystem

    public double currentAngle;
    

    /** Place code here to initialize subsystem */
    public Turret() {
        
        
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
       
        
    }

    public void setAngle(double angle){
        angle = angle/5/18*120;
        if(angle >= 90){
            IntakeMotor.getEncoder().setPosition(90);
        } else if(angle <= -90){
            IntakeMotor.getEncoder().setPosition(-90);
        } else {
            IntakeMotor.getEncoder().setPosition(angle);
        }
       
    }

    public void getAngle(){
        currentAngle = IntakeMotor.getEncoder().getPosition()*5*120/18;
    }

    // place special subsystem methods here
    // this is where rest of program can access functions to return
    // values or control the subsystem


}
