package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
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
        if (masterMotor != null) {
            masterMotor.set(RobotMap.Uptake.UPTAKE_SPEED);
        }
    }

    /**
     * Stops the uptake motors.
     */
    public void stop() {
        if (masterMotor != null) {
            masterMotor.set(0.0);
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
