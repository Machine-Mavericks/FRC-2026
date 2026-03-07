package frc.robot.subsystems;

import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;


/** Subsystem */
public class Intake extends SubsystemBase {

    // Local objects and variables here
    // These are for things that only belong to, and used by, the subsystem

    @Logged
    public double currentPower;
    private double commandedPower;
    private SparkMax intakeMotor;

    /** Place code here to initialize subsystem */
    public Intake() {
     intakeMotor = new SparkMax(RobotMap.CANID.INTAKE_MOTOR, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
        currentPower = intakeMotor.get();
    }

    // place special subsystem methods here
    // this is where rest of program can access functions to return
    // values or control the subsystem
    public void spin(double power){
        this.commandedPower = power;
        intakeMotor.set(power);
    }

    /*
     * returns motor power 1 to -1
     */
    public double getPower(){
        return currentPower;
    }
}
