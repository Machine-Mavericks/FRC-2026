package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/** Subsystem */
public class Climber extends SubsystemBase {

    TalonFX climberMotor = new TalonFX(RobotMap.CANID.CLIMB_MOTOR);
    @Logged
    private double commandedPose;
    public double currentPose;

    private final MotionMagicVoltage mmRequest;
    // Local objects and variables here
    // These are for things that only belong to, and used by, the subsystem

    /** Place code here to initialize subsystem */
    public Climber() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = 12.0;
        config.Slot0.kD = 0.1;
        config.MotionMagic.MotionMagicCruiseVelocity = 5;
        config.MotionMagic.MotionMagicAcceleration = 10;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        mmRequest = new MotionMagicVoltage(0.0);

    }

    /**
     * Method called periodically by the scheduler
     * Place any code here you wish to have run periodically
     */
    @Override
    public void periodic() {

    }

    // place special subsystem methods here
    // this is where rest of program can access functions to return
    // values or control the subsystem

    public void climberDown() {

        

    }

    public void climberUp() {

        

    }
}

