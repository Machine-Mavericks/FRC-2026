package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;


/** Subsystem */
public class LED extends SubsystemBase {

     AddressableLED led;
     AddressableLEDBuffer buffer;

     // used for simple light strobe
    private int StrobeIndex = 0;

    // timer to slow down strobe
    private int timer;

    // Local objects and variables here
    // These are for things that only belong to, and used by, the subsystem

    /** Place code here to initialize subsystem */
    public LED() {
        led = new AddressableLED(RobotMap.PWMPorts.LED_ID);
        led.setLength(60); // 60 LEDs 
        buffer = new AddressableLEDBuffer(60);
        led.setData(buffer);
        led.start();
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {
          timer++;
        if (timer >= 1){
            timer = 0;
        }
        if (timer == 0) {
            StrobeIndex ++;
            if (StrobeIndex >= 60){
                StrobeIndex = 0;
            }
        
            for (int i = 0; i < 60; ++i){
                buffer.setRGB(i, 0, 0, 0);
            }            
        }

        if (RobotContainer.limelightShooter.isTargetPresent()){
                buffer.setRGB(StrobeIndex, 255, 255, 255);
                if (StrobeIndex>0)
                    buffer.setRGB(StrobeIndex-1, 255, 255, 255);
                if (StrobeIndex>1)
                    buffer.setRGB(StrobeIndex-2, 255, 255, 255);
                if (StrobeIndex>2)
                    buffer.setRGB(StrobeIndex-3, 255, 255, 255);
        }else{
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                buffer.setRGB(StrobeIndex, 0, 0, 255);
                if (StrobeIndex>0)
                    buffer.setRGB(StrobeIndex-1, 0, 0, 255);
                if (StrobeIndex>1)
                    buffer.setRGB(StrobeIndex-2, 0, 0, 255);
                if (StrobeIndex>2)
                    buffer.setRGB(StrobeIndex-3, 0, 0, 255);
            }else {
                buffer.setRGB(StrobeIndex, 255, 0, 0);
                if (StrobeIndex>0)
                    buffer.setRGB(StrobeIndex-1, 255, 0, 0);
                if (StrobeIndex>1)
                    buffer.setRGB(StrobeIndex-2, 255, 0, 0);
                if (StrobeIndex>2)
                    buffer.setRGB(StrobeIndex-3, 255, 0, 0);
            }
        }
        
        led.setData(buffer);
        
    }

    // place special subsystem methods here
    // this is where rest of program can access functions to return
    // values or control the subsystem


}
