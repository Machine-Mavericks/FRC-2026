package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/**
 * Command for manual turret control using joystick input.
 * Provides direct speed control with safety limits.
 */
public class ManualTurretControl extends Command {
    
    /**
     * Create the manual turret control command.
     */
    public ManualTurretControl() {
        // Require both turret subsystems
        addRequirements(RobotContainer.turretLeft, RobotContainer.turretRight);
    }
    
    @Override
    public void initialize() {
        // Enable manual control mode
        RobotContainer.turretLeft.enableManualControl(true);
        RobotContainer.turretRight.enableManualControl(true);
        
        SmartDashboard.putString("Turret Mode", "Manual Control");
    }
    
    @Override
    public void execute() {
        // Get joystick inputs from operator controller
        // Left stick Y-axis for left turret, Right stick Y-axis for right turret
        double leftSpeed = -RobotContainer.toolOp.getLeftY(); // Negate for intuitive control
        double rightSpeed = -RobotContainer.toolOp.getRightY();
        
        // Apply deadband
        leftSpeed = applyDeadband(leftSpeed, 0.1);
        rightSpeed = applyDeadband(rightSpeed, 0.1);
        
        // Set turret speeds
        RobotContainer.turretLeft.setManualSpeed(leftSpeed);
        RobotContainer.turretRight.setManualSpeed(rightSpeed);
        
        // Update telemetry
        SmartDashboard.putNumber("Manual Left Speed", leftSpeed);
        SmartDashboard.putNumber("Manual Right Speed", rightSpeed);
    }
    
    /**
     * Apply deadband to joystick input.
     * 
     * @param value Input value
     * @param deadband Deadband threshold
     * @return Processed value
     */
    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        // Scale the output to maintain smooth control
        return (value - Math.signum(value) * deadband) / (1.0 - deadband);
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop turrets
        RobotContainer.turretLeft.setManualSpeed(0.0);
        RobotContainer.turretRight.setManualSpeed(0.0);
        
        // Disable manual control mode
        RobotContainer.turretLeft.enableManualControl(false);
        RobotContainer.turretRight.enableManualControl(false);
        
        SmartDashboard.putString("Turret Mode", "Auto Tracking");
    }
    
    @Override
    public boolean isFinished() {
        // Command runs as long as the button is held
        return false;
    }
}
