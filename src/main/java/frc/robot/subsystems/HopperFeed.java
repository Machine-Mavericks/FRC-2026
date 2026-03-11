package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

/**
 * HopperFeed subsystem - feeds game pieces from the hopper toward the shooter.
 * Single TalonFX with a 1:1 gear ratio. Runs continuously while the robot
 * is enabled at a speed configurable via Shuffleboard.
 */
public class HopperFeed extends SubsystemBase {

    private final TalonFX motor;
    private final GenericEntry speedEntry;

    public HopperFeed() {
        motor = new TalonFX(RobotMap.CANID.HOPPER_FEED);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 1;
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        StatusCode status = motor.getConfigurator().apply(config);

        if (!status.isOK()) {
            System.out.println("Could not apply config: " + status.getName());
        }

        ShuffleboardTab tab = Shuffleboard.getTab("HopperFeed");
        speedEntry = tab.add("Feed Speed", RobotMap.HopperFeed.DEFAULT_SPEED)
                .withPosition(0, 0)
                .withSize(2, 1)
                .getEntry();
    }

    @Override
    public void periodic() {

    }

    /** Returns motor velocity in RPS — useful for diagnostics and smoke tests. */
    public double getVelocityRPS() {
        return motor.getVelocity().getValueAsDouble();
    }

    public void feed() {
        motor.set(RobotMap.HopperFeed.DEFAULT_SPEED);
    }

    public void stop() {
        motor.set(0.0);
    }
}
