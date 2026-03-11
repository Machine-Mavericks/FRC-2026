package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/**
 * Uptake subsystem for feeding game pieces into the shooter flywheels.
 * Uses two TalonFX motors (Falcon 500s) with a 9:1 gear ratio.
 */
public class Uptake extends SubsystemBase {
    private final TalonFX masterMotor;
    private final TalonFX followerMotor;

    public Uptake() {
        this(false);
    }

    /**
     * Create an Uptake, optionally skipping hardware initialization. When
     * skipHardware is true, no TalonFX objects will be created.
     */
    public Uptake(boolean skipHardware) {
        if (!skipHardware) {
            masterMotor = new TalonFX(RobotMap.CANID.UPTAKE_MASTER);
            followerMotor = new TalonFX(RobotMap.CANID.UPTAKE_FOLLOWER);

            TalonFXConfiguration config = new TalonFXConfiguration();
            config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 1;
            config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1;
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
           masterMotor.getConfigurator().apply(config);
           followerMotor.getConfigurator().apply(config);

            // Geared 9:1, follower follows master
            followerMotor.setControl(new Follower(masterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        } else {
            masterMotor = null;
            followerMotor = null;
        }
    }

    /**
     * Runs the uptake forward to feed a ball into the shooter.
     */
    public void feedShooter() {
         masterMotor.set(RobotMap.Uptake.UPTAKE_SPEED);
        
    }

    /**
     * Stops the uptake motors.
     */
    public void stop() {
       
            masterMotor.set(0.0);
    }

    /** Returns master motor velocity in RPS — useful for diagnostics and smoke tests. Returns 0 if hardware is absent. */
    public double getVelocityRPS() {
        if (masterMotor == null) return 0.0;
        return masterMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
