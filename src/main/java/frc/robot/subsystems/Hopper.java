package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/** Subsystem */
public class Hopper extends SubsystemBase {

    TalonFX hopperMotorRight = new TalonFX(RobotMap.CANID.HOPPER_RIGHT);
    TalonFX hopperMotorLeft = new TalonFX(RobotMap.CANID.HOPPER_LEFT);
    @Logged
    private double commandedPose;
    public double currentPose;

    private final MotionMagicVoltage mmRequest;
    // Local objects and variables here
    // These are for things that only belong to, and used by, the subsystem

    /** Place code here to initialize subsystem */
    public Hopper() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = 12.0;
        config.Slot0.kD = 0.1;
        config.MotionMagic.MotionMagicCruiseVelocity = 5;
        config.MotionMagic.MotionMagicAcceleration = 10;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        hopperMotorLeft.setControl(new Follower(hopperMotorRight.getDeviceID(), MotorAlignmentValue.Opposed));
        hopperMotorRight.getConfigurator().apply(config);

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

    public void downHopper() {
        hopperMotorRight.set(-RobotMap.Hopper.HOPPER_SPEED);
    }

    public void upHopper() {
        hopperMotorRight.set(RobotMap.Hopper.HOPPER_SPEED);
    }

    public void stop() {
        hopperMotorRight.set(0);
    }
}
