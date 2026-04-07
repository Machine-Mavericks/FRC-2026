package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.TalonLogger;

/**
 * Intake subsystem for controlling the intake mechanism.
 * Uses two 1:1 geared motors.
 */
public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor_r;
    private final TalonFX followerMotor;

    public IntakeSubsystem() {
        intakeMotor_r = new TalonFX(RobotMap.CANID.INTAKE_SPIN_R);
        intakeMotor_r.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        followerMotor = new TalonFX(RobotMap.CANID.INTAKE_SPIN_L);
        followerMotor.setControl(new Follower(intakeMotor_r.getDeviceID(), MotorAlignmentValue.Opposed));


        // 1:1 geared, follower follows master
        // The second parameter is whether the follower should invert its
        // direction compared to the master.
        // followerMotor.setControl(new Follower(masterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        
        SmartDashboard.putData("Intake/Motor", new TalonLogger(intakeMotor_r));
    }

    // 1:1 geared, follower follows master
    // The second parameter is whether the follower should invert its
    // direction compared to the master.

  /**
   * Create a Shooter, optionally skipping hardware initialization.
   */
  
  public IntakeSubsystem(boolean skipHardware) {
    if (!skipHardware) {
      intakeMotor_r = new TalonFX(RobotMap.CANID.INTAKE_SPIN_R);
        followerMotor = new TalonFX(RobotMap.CANID.INTAKE_SPIN_L);

      TalonFXConfiguration config = new TalonFXConfiguration()
          .withFeedback(
              new FeedbackConfigs())
                  // CTRE Needs reduction ration (N:1) instead of actual ratio
                  //.withSensorToMechanismRatio(1 / MECHANISM_RATIO))
          //.withMotorOutput(new MotorOutputConfigs()
             // .withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Coast))
          //.withSlot0(new Slot0Configs().withKP(1.25) // was 1.25
              //.withKI(2.0).withKD(0).withKV(FEEDFORWARD))
          .withOpenLoopRamps(new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(Seconds.of(5)));
      
      config.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
      intakeMotor_r.getConfigurator().apply(config);
      followerMotor.getConfigurator().apply(config);
      followerMotor.setControl(new Follower(intakeMotor_r.getDeviceID(), MotorAlignmentValue.Opposed));
    } else {
        intakeMotor_r = null; 
          followerMotor = null; 
    }
  }

    /**
     * Runs the intake to pull game pieces in.
     */
    public void intake() {
        intakeMotor_r.set(RobotMap.Intake.INTAKE_SPEED);
    }

    /**
     * Runs the intake to eject game pieces.
     */
    public void outtake() {
        intakeMotor_r.set(RobotMap.Intake.OUTTAKE_SPEED);
    }

    /**
     * Stops the intake motors.
     */
    public void stop() {
        intakeMotor_r.set(0.0);
    }

    /**
     * Returns master motor velocity in RPS — useful for diagnostics and smoke
     * tests.
     */
    public double getVelocityRPS() {
        return intakeMotor_r.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
