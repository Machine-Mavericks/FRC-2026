package com.revrobotics;

public class CANSparkMax {
    public enum SoftLimitDirection { kForward, kReverse }

    private final int id;
    private final CANSparkLowLevel.MotorType motorType;
    private final RelativeEncoder encoder = new RelativeEncoder();
    private final SparkPIDController pid = new SparkPIDController();

    public CANSparkMax(int id, CANSparkLowLevel.MotorType type) {
        this.id = id;
        this.motorType = type;
    }

    public void restoreFactoryDefaults() {}
    public void setIdleMode(CANSparkBase.IdleMode mode) {}
    public void setSmartCurrentLimit(int limit) {}
    public RelativeEncoder getEncoder() { return encoder; }
    public SparkPIDController getPIDController() { return pid; }
    public void enableSoftLimit(SoftLimitDirection dir, boolean enable) {}
    public void setSoftLimit(SoftLimitDirection dir, float value) {}
    public void burnFlash() {}
    public void set(double speed) {}
    public double getOutputCurrent() { return 0.0; }
    public double getMotorTemperature() { return 0.0; }

    // Minimal SoftLimitDirection holder for compatibility
    public static class SoftLimitDirections {
        public static final SoftLimitDirection kForward = SoftLimitDirection.kForward;
        public static final SoftLimitDirection kReverse = SoftLimitDirection.kReverse;
    }
}
