package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/** IntakeArm subsystem */
public class IntakeArm extends SubsystemBase {

    TalonFX intakeArmMotorRight = new TalonFX(RobotMap.CANID.INTAKE_ARM_RIGHT);
    TalonFX intakeArmMotorLeft = new TalonFX(RobotMap.CANID.INTAKE_ARM_LEFT);
    @Logged
    private double commandedPose;
    @Logged
    public double currentPose;
    @Logged
    private double statorCurrent;

    private final MotionMagicVoltage mmRequest;
    // Local objects and variables here
    // These are for things that only belong to, and used by, the subsystem

    /** Place code here to initialize subsystem */
    public IntakeArm() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = 12.0;
        config.Slot0.kD = 0.1;
        config.MotionMagic.MotionMagicCruiseVelocity = 5;
        config.MotionMagic.MotionMagicAcceleration = 10;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Soft limits to constrain the finite range of arm travel
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = RobotMap.IntakeArm.FORWARD_SOFT_LIMIT;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = RobotMap.IntakeArm.REVERSE_SOFT_LIMIT;

        // Stator current limit to protect motor from stall at hard stops
        config.CurrentLimits.StatorCurrentLimit = RobotMap.IntakeArm.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        intakeArmMotorLeft.setControl(new Follower(intakeArmMotorRight.getDeviceID(), MotorAlignmentValue.Opposed));
        StatusCode status = intakeArmMotorRight.getConfigurator().apply(config);
        if (!status.isOK()) {
            System.out.println("Could not apply config: " + status.getName());
        }

        mmRequest = new MotionMagicVoltage(0.0);
    }

    /**
     * Method called periodically by the scheduler
     * Place any code here you wish to have run periodically
     */
    @Override
    public void periodic() {
        currentPose = intakeArmMotorRight.getPosition().getValueAsDouble();
        statorCurrent = intakeArmMotorRight.getStatorCurrent().getValueAsDouble();
    }

    // place special subsystem methods here
    // this is where rest of program can access functions to return
    // values or control the subsystem

    /** Move to a specific position (rotations) using Motion Magic */
    public void moveTo(double position) {
        commandedPose = position;
        intakeArmMotorRight.setControl(mmRequest.withPosition(position));
    }

    public void downIntakeArm() {
        intakeArmMotorRight.set(-RobotMap.IntakeArm.INTAKE_ARM_SPEED);
    }

    public void upIntakeArm() {
        intakeArmMotorRight.set(RobotMap.IntakeArm.INTAKE_ARM_SPEED);
    }

    public void stop() {
        intakeArmMotorRight.set(0);
    }

    /**
     * Zero the encoder at the current position — call when arm is at a known home
     */
    public void zeroEncoder() {
        intakeArmMotorRight.setPosition(0.0);
    }

    /** Returns true if the forward (deployed) hardware limit switch is triggered */
    public boolean isAtForwardLimit() {
        return intakeArmMotorRight.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }

    /**
     * Returns true if the reverse (retracted) hardware limit switch is triggered
     */
    public boolean isAtReverseLimit() {
        return intakeArmMotorRight.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }
}
