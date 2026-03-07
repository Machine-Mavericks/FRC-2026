package com.revrobotics;

public class CANSparkBase {
    public enum ControlType {
        kDutyCycle,
        kPosition,
        kVelocity
    }

    public enum IdleMode {
        kCoast,
        kBrake
    }
}
