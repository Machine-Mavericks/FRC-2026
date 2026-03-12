package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/**
 * Intake subsystem for controlling the intake mechanism.
 * Uses two 1:1 geared motors.
 */
public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX masterMotor;
    // private final TalonFX followerMotor;

    public IntakeSubsystem() {
        masterMotor = new TalonFX(RobotMap.CANID.INTAKE_MASTER);
        masterMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        // followerMotor = new TalonFX(RobotMap.CANID.INTAKE_FOLLOWER);

        // 1:1 geared, follower follows master
        // The second parameter is whether the follower should invert its
        // direction compared to the master.
        // followerMotor.setControl(new Follower(masterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    /**
     * Runs the intake to pull game pieces in.
     */
    public void intake() {
        masterMotor.set(RobotMap.Intake.INTAKE_SPEED);
    }

    /**
     * Runs the intake to eject game pieces.
     */
    public void outtake() {
        masterMotor.set(RobotMap.Intake.OUTTAKE_SPEED);
    }

    /**
     * Stops the intake motors.
     */
    public void stop() {
        masterMotor.set(0.0);
    }

    /** Returns master motor velocity in RPS — useful for diagnostics and smoke tests. */
    public double getVelocityRPS() {
        return masterMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
