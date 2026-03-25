package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.TalonLogger;

public class Shooter extends SubsystemBase {

  TalonFX shooterMotor;

  public double velocity;

  public double wheelRPM;

  public double commanded;

  public double currentSpeed;

  public double targetSpeed;

  public InvertedValue inverted;

  /**
   * Total ratio from motor to flywheel <br/>
   * Represents how many wheel rotations occur for one motor rotation <br/>
   * 12T pulley to 24T pulley, then 30T gear to 26T gear
   */
  private static final double MECHANISM_RATIO = (12 / 24d) * (30 / 26d);

  /**
   * Feedforward value, in RPS per Volt
   */
  private static final double FEEDFORWARD = 0.013;

  private final String basename;

  public Shooter(int shooterMotorId, InvertedValue inverted) {
    this(false, shooterMotorId, inverted);
  }

  /**
   * Create a Shooter, optionally skipping hardware initialization.
   */
  public Shooter(boolean skipHardware, int shooterMotorId, InvertedValue inverted) {
    basename = "Shooter [" + shooterMotorId + "]";
    if (!skipHardware) {
      shooterMotor = new TalonFX(shooterMotorId);
      
      SmartDashboard.putData(basename+"shooter/Motor", new TalonLogger(shooterMotor));

      TalonFXConfiguration config = new TalonFXConfiguration()
          .withFeedback(
              new FeedbackConfigs()
                  // CTRE Needs reduction ration (N:1) instead of actual ratio
                  .withSensorToMechanismRatio(1 / MECHANISM_RATIO))
          .withMotorOutput(new MotorOutputConfigs()
              .withInverted(inverted).withNeutralMode(NeutralModeValue.Coast))
          .withSlot0(new Slot0Configs().withKP(1.25) // was 1.25
              .withKI(2.0).withKD(0).withKV(FEEDFORWARD))
          .withClosedLoopRamps(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(Seconds.of(1)));

      StatusCode status = shooterMotor.getConfigurator().apply(config);
      if (!status.isOK()) {
        System.out.println("Could not apply config: " + status.getName());
      }
    } else {
      shooterMotor = null;
    }
  }

  public void shooterSpeed(double speed) {
    SmartDashboard.putNumber(basename+"/desired", speed);
    if (shooterMotor != null) {
      if (speed < 15) { // in RPS
        shooterMotor.set(0);
      } else {
        shooterMotor.setControl(new VelocityVoltage(speed));
      }
    }

    // Always update bookkeeping fields even when hardware is absent
    commanded = speed;
  }

  public void stop() {
    if (shooterMotor != null) {
      shooterMotor.set(0);
    }

    commanded = 0;
  }

  public double CalculateSpeed(double distance) { // saying its metres
    double x = distance;
    return 1.266 * x * x - 4.1598 * x + 49.3;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(basename+"shooter/actual", shooterMotor.getVelocity().getValueAsDouble());
    
    if (shooterMotor != null) {
      currentSpeed = shooterMotor.get();

      // Get velocity in rotations per second (RPS) from the Motor
      double velocity = shooterMotor.getVelocity().getValueAsDouble();
      double wheelRPM = velocity * 60;

      this.velocity = velocity;
      this.wheelRPM = wheelRPM;
      
    } else {
      // No hardware: keep existing internal values
    }
  }

  // IDK if we are actually gonna do simulation but ill leave it here just in case
  @Override
  public void simulationPeriodic() {
    // Code here would run each cycle while simulating the robot.
  }

  public double getBolts() {
    if (shooterMotor != null) {
      return shooterMotor.getMotorVoltage().getValueAsDouble();
    }
    return 0.0;
  }

  public double getBusVolts() {
    if (shooterMotor != null) {
      return shooterMotor.getSupplyVoltage().getValueAsDouble();
    }
    return 0.0;
  }

  public double getCurrent() {
    if (shooterMotor != null) {
      return shooterMotor.getStatorCurrent().getValueAsDouble();
    }
    return 0.0;
  }

  public double getVelocity() {
    return velocity;
  }

  public double getWheelRPM() {
    return wheelRPM;
  }

  public double getTargetSpeed() {
    return targetSpeed;
  }
}
