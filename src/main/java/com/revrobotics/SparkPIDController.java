package com.revrobotics;

public class SparkPIDController {
    private double p, i, d, iZone, ff;
    private double minOutput = -1.0, maxOutput = 1.0;

    public void setP(double p) { this.p = p; }
    public void setI(double i) { this.i = i; }
    public void setD(double d) { this.d = d; }
    public void setIZone(double iz) { this.iZone = iz; }
    public void setFF(double ff) { this.ff = ff; }
    public void setOutputRange(double min, double max) { this.minOutput = min; this.maxOutput = max; }

    public void setReference(double value, CANSparkBase.ControlType type) {
        // no-op stub
    }
}
