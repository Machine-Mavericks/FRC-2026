package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.epilogue.Logged;
// import edu.wpi.first.wpilibj.DigitalInput;  // Uncomment when limit switches are installed
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/** Subsystem for the Hopper arm, which has two positions: DOWN and UP */
public class Hopper extends SubsystemBase {

    // -------------------------------------------------------------------------
    // Motor Controllers
    // -------------------------------------------------------------------------
    TalonFX hopperMotorRight = new TalonFX(RobotMap.CANID.HOPPER_RIGHT);
    TalonFX hopperMotorLeft = new TalonFX(RobotMap.CANID.HOPPER_LEFT);

    // -------------------------------------------------------------------------
    // Arm Position Constants (in rotations — tune these to your robot)
    // -------------------------------------------------------------------------
    private static final double POSITION_DOWN = 0.0; // TODO: tune for down position
    private static final double POSITION_UP = 10.0; // TODO: tune for up position

    // -------------------------------------------------------------------------
    // Limit Switches (commented out — not yet installed on robot)
    // -------------------------------------------------------------------------
    // private final DigitalInput limitSwitchDown = new
    // DigitalInput(RobotMap.DIO.HOPPER_LIMIT_DOWN);
    // private final DigitalInput limitSwitchUp = new
    // DigitalInput(RobotMap.DIO.HOPPER_LIMIT_UP);

    // -------------------------------------------------------------------------
    // State tracking
    // -------------------------------------------------------------------------
    @Logged
    private double commandedPose;
    public double currentPose;

    private final MotionMagicVoltage mmRequest;

    // -------------------------------------------------------------------------
    /** Initializes the Hopper subsystem */
    public Hopper() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = 12.0;
        config.Slot0.kD = 0.1;
        config.MotionMagic.MotionMagicCruiseVelocity = 5;
        config.MotionMagic.MotionMagicAcceleration = 10;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        hopperMotorRight.getConfigurator().apply(config);
        hopperMotorLeft.setControl(new Follower(hopperMotorRight.getDeviceID(), MotorAlignmentValue.Opposed));

        mmRequest = new MotionMagicVoltage(0.0);
    }

    // -------------------------------------------------------------------------
    /**
     * Called periodically by the scheduler.
     * Update current position and check limit switches when they are installed.
     */
    @Override
    public void periodic() {
        currentPose = hopperMotorRight.getPosition().getValueAsDouble();

        // -- Limit switch logic (uncomment when switches are installed) --------
        // if (!limitSwitchDown.get()) {
        // // At the down hard stop — reset encoder and stop motion
        // hopperMotorRight.setPosition(POSITION_DOWN);
        // }
        // if (!limitSwitchUp.get()) {
        // // At the up hard stop — reset encoder and stop motion
        // hopperMotorRight.setPosition(POSITION_UP);
        // }
        // -----------------------------------------------------------------------
    }

    // -------------------------------------------------------------------------
    // Public Commands
    // -------------------------------------------------------------------------

    /**
     * Moves the hopper arm to the DOWN position using Motion Magic.
     */
    public void downHopper() {
        commandedPose = POSITION_DOWN;
        hopperMotorRight.setControl(mmRequest.withPosition(commandedPose));
    }

    /**
     * Moves the hopper arm to the UP position using Motion Magic.
     */
    public void upHopper() {
        commandedPose = POSITION_UP;
        hopperMotorRight.setControl(mmRequest.withPosition(commandedPose));
    }

    // -------------------------------------------------------------------------
    // Status / Helpers
    // -------------------------------------------------------------------------

    /**
     * Returns true when the arm is within tolerance of the DOWN position.
     * Once limit switches are installed, also check limitSwitchDown.
     */
    public boolean isDown() {
        return Math.abs(currentPose - POSITION_DOWN) < 0.5;
        // && !limitSwitchDown.get() // uncomment when limit switch is installed
    }

    /**
     * Returns true when the arm is within tolerance of the UP position.
     * Once limit switches are installed, also check limitSwitchUp.
     */
    public boolean isUp() {
        return Math.abs(currentPose - POSITION_UP) < 0.5;
        // && !limitSwitchUp.get() // uncomment when limit switch is installed
    }
}
