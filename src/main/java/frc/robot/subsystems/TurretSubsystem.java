package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/**
 * Base class for turret subsystem.
 * Controls a single turret axis using a NEO 550 motor with SparkMax controller.
 */
public abstract class TurretSubsystem extends SubsystemBase {
    
    protected final SparkMax motor;
    protected final RelativeEncoder encoder;
    // Use a software PIDController (runs on RoboRIO) to control turret position
    protected final PIDController pidController;
    
    protected final String name;
    protected double targetAngleDegrees = 0.0;
    protected boolean manualControlEnabled = false;
    private static final double MECHANISM_RATIO = (1.0 / 5.0) * (18.0 / 120.0);
    // Shuffleboard
    protected ShuffleboardTab tab;
    
    /**
     * Create a turret subsystem.
     * 
     * @param motorCANID CAN ID for the turret motor
     * @param name Name of this turret (for telemetry)
     */
    /**
     * Primary constructor. Use this when hardware is present.
     */
    public TurretSubsystem(int motorCANID, String name) {
        this(false, motorCANID, name);
    }

    /**
     * Constructor that can skip hardware initialization when running on a robot
     * that doesn't have this motor/controller installed. When skipHardware is
     * true, motor/encoder/pidController will be left null and subclasses should
     * avoid calling hardware methods.
     */
    public TurretSubsystem(boolean skipHardware, int motorCANID, String name) {
        this.name = name;

        if (!skipHardware) {
            // Initialize motor (REVLib new API)
            motor = new SparkMax(motorCANID, MotorType.kBrushless);
            SparkMaxConfig config = new SparkMaxConfig();
            config.encoder.positionConversionFactor(MECHANISM_RATIO);
            config.openLoopRampRate(0.5);
            config.smartCurrentLimit(10);

            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            // Get encoder (returns rotations) and create a software PID controller
            encoder = motor.getEncoder();
            resetEncoder();
            pidController = new PIDController(RobotMap.Turret.kP, RobotMap.Turret.kI, RobotMap.Turret.kD);
            pidController.setTolerance(RobotMap.Turret.POSITION_TOLERANCE);
            pidController.setIntegratorRange(-0.1, 0.1);

            // Initialize Shuffleboard
            initializeShuffleboard();
        } else {
            // Hardware skipped; leave motor/encoder/pidController null
        motor = null;
        encoder = null;
        // create a dummy PIDController to keep final field semantics
        pidController = null;
            // Subclasses (disabled stubs) must avoid calling hardware methods
            // and should override telemetry methods as needed.
            // Note: keep fields final by assigning null here.
            // (They were declared final; Java allows final fields to be assigned
            // in constructor.)
            // no-op
            // initializeShuffleboard intentionally not called
            // Assign nulls to final fields via reflection not needed; assign here
            // via local initialization below
            // But since fields are final, we must assign them. Use simple approach:
        }
    }

    
    /**
     * Set the target angle for the turret.
     * 
     * @param angleDegrees Target angle in degrees (0 = forward, positive = counterclockwise)
     */
    public void setTargetAngle(double angleDegrees) {
        // Clamp to limits
        angleDegrees = Math.max(RobotMap.Turret.MIN_ROTATION_DEGREES, 
                               Math.min(RobotMap.Turret.MAX_ROTATION_DEGREES, angleDegrees));
        
        targetAngleDegrees = angleDegrees;

        SmartDashboard.putNumber("Target Angle", targetAngleDegrees);
        
        if (!manualControlEnabled && pidController != null) {
            // Run software PID: set the setpoint (degrees). Actual motor output
            // is applied in periodic().
            pidController.setSetpoint(angleDegrees);
        }
    }
    
    /**
     * Get the current turret angle.
     * 
     * @return Current angle in degrees
     */
    public double getCurrentAngle() {
        if (encoder != null) {
            // encoder returns rotations; convert to degrees accounting for gear ratio
            return encoder.getPosition() * 360.0;
        }
        return targetAngleDegrees;
    }
    
    /**
     * Check if the turret is at the target position.
     * 
     * @return True if within tolerance of target
     */
    public boolean atSetpoint() {
        return Math.abs(getCurrentAngle() - targetAngleDegrees) < RobotMap.Turret.POSITION_TOLERANCE;
    }
    
    /**
     * Enable manual control mode.
     * When enabled, direct speed control is used instead of PID position control.
     * 
     * @param enabled True to enable manual control
     */
    public void enableManualControl(boolean enabled) {
        manualControlEnabled = enabled;
        if (!enabled) {
            // Re-engage PID when exiting manual mode
            setTargetAngle(targetAngleDegrees);
        }
    }
    
    /**
     * Set manual control speed.
     * Only active when manual control is enabled.
     * 
     * @param speed Speed as percent output (-1.0 to 1.0)
     */
    public void setManualSpeed(double speed) {
        if (manualControlEnabled) {
            // Clamp speed
            speed = Math.max(-RobotMap.Turret.MANUAL_SPEED_LIMIT, 
                           Math.min(RobotMap.Turret.MANUAL_SPEED_LIMIT, speed));
            
            // Additional safety: stop at limits
            double currentAngle = getCurrentAngle();
            if ((currentAngle >= RobotMap.Turret.MAX_ROTATION_DEGREES && speed > 0) ||
                (currentAngle <= RobotMap.Turret.MIN_ROTATION_DEGREES && speed < 0)) {
                speed = 0.0;
            }
            
            if (motor != null) {
                motor.set(speed);
            }
        }
    }
    
    /**
     * Reset the encoder to zero at the current position.
     */
    public void resetEncoder() {
        if (encoder != null) {
            encoder.setPosition(0.0);
        }
        targetAngleDegrees = 0.0;
    }
    
    /**
     * Get the target angle.
     * @return Target angle in degrees
     */
    public double getTargetAngle() {
        return targetAngleDegrees;
    }
    
    @Override
    public void periodic() {
        // If not in manual control, run software PID to compute motor output
        if (!manualControlEnabled && pidController != null && motor != null && encoder != null) {
            double output = pidController.calculate(getCurrentAngle());
            // Clamp output to percent output range
            double speedLimit = 0.2;
            output = Math.max(-speedLimit, Math.min(speedLimit  , output));
            // Safety limits: don't drive past physical limits
            double currentAngle = getCurrentAngle();
            if ((currentAngle >= RobotMap.Turret.MAX_ROTATION_DEGREES && output > 0) ||
                (currentAngle <= RobotMap.Turret.MIN_ROTATION_DEGREES && output < 0)) {
                output = 0.0;
            }
            motor.set(output);

            SmartDashboard.putNumber("Turret/CurrentAngle", currentAngle);
            SmartDashboard.putNumber("Output", output);
            SmartDashboard.putNumber("Setpoint", pidController.getSetpoint());
        }

        updateShuffleboard();
    }
    
    /**
     * Initialize Shuffleboard telemetry.
     */
    protected void initializeShuffleboard() {
        tab = Shuffleboard.getTab(name);
        tab.addNumber("Current Angle (deg)", this::getCurrentAngle);
        tab.addNumber("Target Angle (deg)", this::getTargetAngle);
        tab.addBoolean("At Setpoint", this::atSetpoint);
        tab.addBoolean("Manual Control", () -> manualControlEnabled);
        if (motor != null) {
            tab.addNumber("Motor Current (A)", motor::getOutputCurrent);
            tab.addNumber("Motor Temp (C)", motor::getMotorTemperature);
        }
    }
    
    /**
     * Update Shuffleboard values.
     */
    protected void updateShuffleboard() {
        // Values are updated automatically via suppliers in initializeShuffleboard
    }
}
