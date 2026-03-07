package com.revrobotics;

public class RelativeEncoder {
    private double position = 0.0;
    private double positionConversion = 1.0;
    private double velocityConversion = 1.0;

    public void setPositionConversionFactor(double factor) {
        this.positionConversion = factor;
    }

    public void setVelocityConversionFactor(double factor) {
        this.velocityConversion = factor;
    }

    public void setPosition(double pos) {
        this.position = pos;
    }

    public double getPosition() {
        return this.position * this.positionConversion;
    }

    public double getVelocity() {
        return 0.0 * this.velocityConversion;
    }
}
