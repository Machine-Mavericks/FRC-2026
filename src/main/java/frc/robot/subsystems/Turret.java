package frc.robot.subsystems;

import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


/** Subsystem */
public class Turret extends SubsystemBase {

    // Local objects and variables here
    // These are for things that only belong to, and used by, the subsystem

    /** Place code here to initialize subsystem */
    public Turret() {
        SparkMax IntakeMotor = new SparkMax(3, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }

    // place special subsystem methods here
    // this is where rest of program can access functions to return
    // values or control the subsystem


}
