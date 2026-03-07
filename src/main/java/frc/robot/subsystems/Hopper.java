package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;

import edu.wpi.first.epilogue.Logged;
// import edu.wpi.first.wpilibj.DigitalInput;  // Uncomment when limit switches are installed
import edu.wpi.first.wpilibj.Timer;
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
    // Gear Reduction & Arm Position Constants
    // -------------------------------------------------------------------------
    // 36.25 motor rotations : 1 arm rotation
    private static final double GEAR_RATIO = 36.25;

    // Positions expressed in ARM (mechanism) rotations — tune to your robot
    private static final double POSITION_DOWN = 0.0; // TODO: tune for down position
    private static final double POSITION_UP = 0.25; // TODO: tune for up position (arm rotations)

    // -------------------------------------------------------------------------
    // Current-Based Stall Detection (substitute for uninstalled limit switches)
    // -------------------------------------------------------------------------
    /**
     * Stator current threshold (amps) above which we consider the arm stalled
     * against a hard stop. Start conservative (~30–40 A) and tune upward if
     * normal acceleration trips it falsely.
     * TODO: tune this value on the actual robot.
     */
    private static final double STALL_CURRENT_THRESHOLD_AMPS = 35.0;

    /**
     * How long (seconds) the current must stay above the threshold before we
     * declare a stall. This prevents brief acceleration spikes from triggering
     * a false stop.
     * TODO: tune this value on the actual robot.
     */
    private static final double STALL_DEBOUNCE_SECONDS = 0.1;

    // Timers: reset when current drops below threshold, start counting when it
    // rises.
    private final Timer stallTimerDown = new Timer();
    private final Timer stallTimerUp = new Timer();

    // Are we currently commanding motion toward each limit?
    private boolean movingDown = false;
    private boolean movingUp = false;

    // Stall flags — set when current debounce fires, cleared when new command
    // issued.
    @Logged
    public boolean stalledAtDown = false;
    @Logged
    public boolean stalledAtUp = false;

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
    @Logged
    public double currentPose;
    @Logged
    public double statorCurrentAmps;

    private final MotionMagicVoltage mmRequest;
    private final NeutralOut stopRequest = new NeutralOut();

    // -------------------------------------------------------------------------
    /** Initializes the Hopper subsystem */
    public Hopper() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Apply gear reduction so all positions/velocities are in arm (mechanism)
        // rotations.
        // Phoenix 6 multiplies internally: motor rotations / GEAR_RATIO = arm
        // rotations.
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        config.Slot0.kP = 12.0;
        config.Slot0.kD = 0.1;
        // Cruise/accel are now in ARM rotations/sec — tune these to your mechanism
        config.MotionMagic.MotionMagicCruiseVelocity = 0.5; // TODO: tune (arm rot/s)
        config.MotionMagic.MotionMagicAcceleration = 1.0; // TODO: tune (arm rot/s²)

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        hopperMotorRight.getConfigurator().apply(config);
        hopperMotorLeft.setControl(new Follower(hopperMotorRight.getDeviceID(), MotorAlignmentValue.Opposed));

        mmRequest = new MotionMagicVoltage(0.0);

        stallTimerDown.reset();
        stallTimerUp.reset();
    }

    // -------------------------------------------------------------------------
    /**
     * Called periodically by the scheduler.
     * Reads position and stator current; runs current-based stall detection
     * as a software stand-in for the uninstalled limit switches.
     */
    @Override
    public void periodic() {
        currentPose = hopperMotorRight.getPosition().getValueAsDouble();
        statorCurrentAmps = hopperMotorRight.getStatorCurrent().getValueAsDouble();

        // -- Current-based stall detection (substitute for limit switches) ------
        runStallDetection();
        // -----------------------------------------------------------------------

        // -- Limit switch logic (uncomment when switches are installed) ---------
        // if (!limitSwitchDown.get()) {
        // hopperMotorRight.setPosition(POSITION_DOWN);
        // stalledAtDown = true;
        // hopperMotorRight.setControl(stopRequest);
        // }
        // if (!limitSwitchUp.get()) {
        // hopperMotorRight.setPosition(POSITION_UP);
        // stalledAtUp = true;
        // hopperMotorRight.setControl(stopRequest);
        // }
        // -----------------------------------------------------------------------
    }

    // -------------------------------------------------------------------------
    // Public Commands
    // -------------------------------------------------------------------------

    /**
     * Moves the hopper arm to the DOWN position using Motion Magic.
     * Ignored if the arm is already stalled at the down limit.
     */
    public void downHopper() {
        stalledAtDown = false; // clear stall flag so a new command is allowed
        movingDown = true;
        movingUp = false;
        stallTimerDown.reset();
        stallTimerDown.start();
        commandedPose = POSITION_DOWN;
        hopperMotorRight.setControl(mmRequest.withPosition(commandedPose));
    }

    /**
     * Moves the hopper arm to the UP position using Motion Magic.
     * Ignored if the arm is already stalled at the up limit.
     */
    public void upHopper() {
        stalledAtUp = false; // clear stall flag so a new command is allowed
        movingDown = false;
        movingUp = true;
        stallTimerUp.reset();
        stallTimerUp.start();
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
        return Math.abs(currentPose - POSITION_DOWN) < 0.5 || stalledAtDown;
        // && !limitSwitchDown.get() // uncomment when limit switch is installed
    }

    /**
     * Returns true when the arm is within tolerance of the UP position.
     * Once limit switches are installed, also check limitSwitchUp.
     */
    public boolean isUp() {
        return Math.abs(currentPose - POSITION_UP) < 0.5 || stalledAtUp;
        // && !limitSwitchUp.get() // uncomment when limit switch is installed
    }

    // -------------------------------------------------------------------------
    // Private Helpers
    // -------------------------------------------------------------------------

    /**
     * Checks stator current against {@link #STALL_CURRENT_THRESHOLD_AMPS} while
     * the arm is in motion. If the current exceeds the threshold for longer than
     * {@link #STALL_DEBOUNCE_SECONDS}, the motor is stopped and the appropriate
     * stall flag is latched.
     *
     * <p>
     * Replace this method (or set the stall flags from limit-switch logic in
     * {@link #periodic()}) once the physical switches are installed.
     */
    private void runStallDetection() {
        boolean overcurrent = statorCurrentAmps > STALL_CURRENT_THRESHOLD_AMPS;

        // --- Down direction ---
        if (movingDown && !stalledAtDown) {
            if (overcurrent) {
                if (stallTimerDown.hasElapsed(STALL_DEBOUNCE_SECONDS)) {
                    stalledAtDown = true;
                    movingDown = false;
                    hopperMotorRight.setControl(stopRequest);
                    // Latch the encoder at the down position so future moves are accurate.
                    hopperMotorRight.setPosition(POSITION_DOWN);
                }
                // else: current is high but hasn't held long enough — keep waiting
            } else {
                // Current dropped back below threshold; keep the timer running from 0
                // so only a sustained overcurrent causes a stop.
                stallTimerDown.reset();
            }
        } else {
            stallTimerDown.reset();
        }

        // --- Up direction ---
        if (movingUp && !stalledAtUp) {
            if (overcurrent) {
                if (stallTimerUp.hasElapsed(STALL_DEBOUNCE_SECONDS)) {
                    stalledAtUp = true;
                    movingUp = false;
                    hopperMotorRight.setControl(stopRequest);
                    // Latch the encoder at the up position so future moves are accurate.
                    hopperMotorRight.setPosition(POSITION_UP);
                }
                // else: current is high but hasn't held long enough — keep waiting
            } else {
                stallTimerUp.reset();
            }
        } else {
            stallTimerUp.reset();
        }
    }
}
