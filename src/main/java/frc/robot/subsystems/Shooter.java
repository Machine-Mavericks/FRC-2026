package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {

  TalonFX shooterMotor = new TalonFX(RobotMap.CANID.SHOOTER);

  public double velocity;

  public double wheelRPM;

  public double commanded;

  public double currentSpeed;

  public double targetSpeed;

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

  public Shooter() {

    TalonFXConfiguration config = new TalonFXConfiguration()
        .withFeedback(
            new FeedbackConfigs()
                // CTRE Needs reduction ration (N:1) instead of actual ratio
                .withSensorToMechanismRatio(1 / MECHANISM_RATIO))
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast))
        .withSlot0(new Slot0Configs()
            .withKP(1.25) // was 1.25
            .withKI(2.0)
            .withKD(0)
            .withKV(FEEDFORWARD))
        .withClosedLoopRamps(new ClosedLoopRampsConfigs()
            .withVoltageClosedLoopRampPeriod(Seconds.of(1)));

    shooterMotor.getConfigurator().apply(config);
  }

  public void shooterSpeed(double speed) {
    if (speed < 15) { // in RPS
      shooterMotor.set(0);
    } else {
      shooterMotor.setControl(new VelocityVoltage(speed));
    }

    commanded = speed;
  }

  public void stop() {
    shooterMotor.set(0);

    commanded = 0;
  }

  public double CalculateSpeed(double distance) { // saying its metres
    double x = distance;
    return 2.9382 * x * x - 7.1015 * x + 51.427;
  }

  @Override
  public void periodic() {

    currentSpeed = shooterMotor.get();

    // Get velocity in rotations per second (RPS) from the Motor
    double velocity = shooterMotor.getVelocity().getValueAsDouble();
    double wheelRPM = velocity * 60;

    this.velocity = velocity;
    this.wheelRPM = wheelRPM;
  }

  // IDK if we are actually gonna do simulation but ill leave it here just in case
  @Override
  public void simulationPeriodic() {
    // Code here would run each cycle while simulating the robot.
  }

  public double getBolts() {
    return shooterMotor.getMotorVoltage().getValueAsDouble();
  }

  public double getBusVolts() {
    return shooterMotor.getSupplyVoltage().getValueAsDouble();
  }

  public double getCurrent() {
    return shooterMotor.getStatorCurrent().getValueAsDouble();
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
