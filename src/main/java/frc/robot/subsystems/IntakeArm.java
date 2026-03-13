package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.revrobotics.spark.FeedbackSensor;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/** IntakeArm subsystem */
public class IntakeArm extends SubsystemBase {

    TalonFX intakeArmMotorRight = new TalonFX(RobotMap.CANID.INTAKE_ARM_RIGHT);
    TalonFX intakeArmMotorLeft = new TalonFX(RobotMap.CANID.INTAKE_ARM_LEFT);

    private static final double MECHANISM_RATIO = (12.0 / 58.0) * (27.0 / 56.0) * (16.0 / 40.0);

    @Logged
    private double commandedPose;
    @Logged
    public double leftCurrentPose;
    @Logged
    public double rightCurrentPose;
    @Logged
    private double statorCurrent;

    // Local objects and variables here
    // These are for things that only belong to, and used by, the subsystem

    /** Place code here to initialize subsystem */
    public IntakeArm() {
        TalonFXConfiguration configLeft = new TalonFXConfiguration();

        configLeft.Slot0.kP = 12.0;
        configLeft.Slot0.kD = 0.1;
        configLeft.Slot0.kG = 0.0; // TUNE: Voltage required to hold arm horizontal
        configLeft.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        configLeft.MotionMagic.MotionMagicCruiseVelocity = 1;
        configLeft.MotionMagic.MotionMagicAcceleration = 2;

        configLeft.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configLeft.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Soft limits to constrain the finite range of arm travel
        configLeft.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configLeft.SoftwareLimitSwitch.ForwardSoftLimitThreshold = RobotMap.IntakeArm.FORWARD_SOFT_LIMIT;
        configLeft.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        configLeft.SoftwareLimitSwitch.ReverseSoftLimitThreshold = RobotMap.IntakeArm.REVERSE_SOFT_LIMIT;

        // Stator current limit to protect motor from stall at hard stops
        configLeft.CurrentLimits.StatorCurrentLimit = RobotMap.IntakeArm.STATOR_CURRENT_LIMIT;
        configLeft.CurrentLimits.StatorCurrentLimitEnable = true;

        configLeft.Feedback.withSensorToMechanismRatio(1 / MECHANISM_RATIO);

        intakeArmMotorLeft.getConfigurator().apply(configLeft);

        TalonFXConfiguration configRight = new TalonFXConfiguration();

        configRight.Slot0.kP = 12.0;
        configRight.Slot0.kD = 0.1;
        configRight.Slot0.kG = 0.0; // TUNE: Voltage required to hold arm horizontal
        configRight.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        configRight.MotionMagic.MotionMagicCruiseVelocity = 1;
        configRight.MotionMagic.MotionMagicAcceleration = 2;

        configRight.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configRight.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Soft limits to constrain the finite range of arm travel
        configRight.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configRight.SoftwareLimitSwitch.ForwardSoftLimitThreshold = RobotMap.IntakeArm.FORWARD_SOFT_LIMIT;
        configRight.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        configRight.SoftwareLimitSwitch.ReverseSoftLimitThreshold = RobotMap.IntakeArm.REVERSE_SOFT_LIMIT;

        // Stator current limit to protect motor from stall at hard stops
        configRight.CurrentLimits.StatorCurrentLimit = RobotMap.IntakeArm.STATOR_CURRENT_LIMIT;
        configRight.CurrentLimits.StatorCurrentLimitEnable = true;

        configRight.Feedback.withSensorToMechanismRatio(1 / MECHANISM_RATIO);

        intakeArmMotorRight.getConfigurator().apply(configRight);

    }

    /**
     * Method called periodically by the scheduler
     * Place any code here you wish to have run periodically
     */
    @Override
    public void periodic() {
        leftCurrentPose = intakeArmMotorLeft.getPosition().getValueAsDouble();
        double leftStatorCurrent = intakeArmMotorLeft.getStatorCurrent().getValueAsDouble();

        rightCurrentPose = intakeArmMotorRight.getPosition().getValueAsDouble();
        double rightStatorCurrent = intakeArmMotorRight.getStatorCurrent().getValueAsDouble();

        SmartDashboard.putNumber("Arm/leftPosition", leftCurrentPose);
        SmartDashboard.putNumber("Arm/rightPosition", rightCurrentPose);
        SmartDashboard.putNumber("Arm/rightCurrent", leftStatorCurrent);
        SmartDashboard.putNumber("Arm/leftCurrent", rightStatorCurrent);

    }

    public void debug(double speed) {
        intakeArmMotorRight.set(speed);
        intakeArmMotorLeft. set(speed);
    }

    // place special subsystem methods here
    // this is where rest of program can access functions to return
    // values or control the subsystem

    private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

    /** Move to a specific position (rotations) using Motion Magic */
    /**
     * Move to a specific position (rotations) using Motion Magic. 
     * Uses a 'Dynamic Leash' for synchronization: allows full Motion Magic profiles
     * when motors are synced, but limits the lead if they drift apart.
     */
    public void moveTo(double position) {
        double diff = rightCurrentPose - leftCurrentPose;
        double targetPosLeft = position;
        double targetPosRight = position;

        // If motors are out of sync by more than ~1 degree (1.0/360.0), 
        // leash the faster motor to stay near the slower one.
        double syncTolerance = 1.0 / 360.0;
        double leashLimit = 5.0 / 360.0;

        if (Math.abs(diff) > syncTolerance) {
            if (position > leftCurrentPose) { // Moving positive
                // Leash the leading motor to stay within 5 degrees of the trailing one
                targetPosLeft = Math.min(position, rightCurrentPose + leashLimit);
                targetPosRight = Math.min(position, leftCurrentPose + leashLimit);
            } else { // Moving negative
                targetPosLeft = Math.max(position, rightCurrentPose - leashLimit);
                targetPosRight = Math.max(position, leftCurrentPose - leashLimit);
            }
        }

        // Clamp positions to soft limits
        targetPosLeft = MathUtil.clamp(targetPosLeft, RobotMap.IntakeArm.REVERSE_SOFT_LIMIT,
                RobotMap.IntakeArm.FORWARD_SOFT_LIMIT);
        targetPosRight = MathUtil.clamp(targetPosRight, RobotMap.IntakeArm.REVERSE_SOFT_LIMIT,
                RobotMap.IntakeArm.FORWARD_SOFT_LIMIT);

        intakeArmMotorLeft.setControl(m_mmReq.withPosition(targetPosLeft));
        intakeArmMotorRight.setControl(m_mmReq.withPosition(targetPosRight));
    }

    /*
     * should be using moveTo() instead with
     * frc.robot.RobotMap.IntakeArm.DEPLOYED_POSITION
     * or frc.robot.RobotMap.IntakeArm.STOWED_POSITION
     */
    // public void downIntakeArm() {
    // intakeArmMotorRight.set(-RobotMap.IntakeArm.INTAKE_ARM_SPEED);
    // }

    // public void upIntakeArm() {
    // intakeArmMotorRight.set(RobotMap.IntakeArm.INTAKE_ARM_SPEED);
    // }

    public void stop() {
        intakeArmMotorRight.set(0);
        intakeArmMotorLeft.set(0);
    }

    /**
     * IMPORTANT: With horizontal = 0, call this when the arm is PERFECTLY
     * HORIZONTAL.
     * Alternatively, if booting up in the Stowed/Up position, use
     * setPosition(RobotMap.IntakeArm.STOWED_POSITION).
     */
    public void zeroEncoder() {
        intakeArmMotorRight.setPosition(0.0);
        intakeArmMotorLeft.setPosition(0.0);
    }

    /**
     * Set the encoder to the STOWED position. Call this if the robot boots up with
     * the arm straight up.
     */
    public void setStowedPosition() {
        intakeArmMotorRight.setPosition(RobotMap.IntakeArm.STOWED_POSITION);
        intakeArmMotorLeft.setPosition(RobotMap.IntakeArm.STOWED_POSITION);
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
