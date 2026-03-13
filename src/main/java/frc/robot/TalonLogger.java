package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class TalonLogger implements Sendable {
    
    private final TalonFX talon;

    public TalonLogger(TalonFX talon){
        this.talon = talon;
    }

    @Override
    public void initSendable(SendableBuilder builder){
        builder.addIntegerProperty("CAN ID",  talon::getDeviceID, null);
        builder.addDoubleProperty("OutputVolts", () -> talon.getMotorVoltage().getValueAsDouble(), null);
        builder.addDoubleProperty("OutputCurrent", () -> talon.getTorqueCurrent().getValueAsDouble(), null);
        builder.addDoubleProperty("position", () -> talon.getPosition().getValueAsDouble(), null);
        builder.addDoubleProperty("velocity", () -> talon.getVelocity().getValueAsDouble(), null);
    }

}