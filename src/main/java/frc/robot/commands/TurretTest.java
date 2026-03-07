package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

/**
 * Calibration command for turret subsystems.
 * Sweeps turrets through their full range of motion and logs encoder values.
 */
public class TurretTest extends Command {
    
    private enum TestPhase {
        ZERO,
        POSITIVE_45,
        POSITIVE_90,
        RETURN_TO_ZERO,
        NEGATIVE_45,
        NEGATIVE_90,
        FINAL_ZERO,
        COMPLETE
    }
    
    private TestPhase phase = TestPhase.ZERO;
    private Timer timer = new Timer();
    private static final double SETTLE_TIME = 1.0; // seconds to wait at each position
    
    /**
     * Create the turret test command.
     */
    public TurretTest() {
        addRequirements(RobotContainer.turretLeft, RobotContainer.turretRight);
    }
    
    @Override
    public void initialize() {
        phase = TestPhase.ZERO;
        timer.reset();
        timer.start();
        
        // Disable manual control
        RobotContainer.turretLeft.enableManualControl(false);
        RobotContainer.turretRight.enableManualControl(false);
        
        SmartDashboard.putString("Turret Test Phase", "Starting");
        System.out.println("=== TURRET CALIBRATION TEST STARTED ===");
    }
    
    @Override
    public void execute() {
        if (timer.hasElapsed(SETTLE_TIME)) {
            // Log current position
            logPosition();
            
            // Advance to next phase
            timer.reset();
            switch (phase) {
                case ZERO:
                    moveToPosition(0.0, "Moving to 0°");
                    phase = TestPhase.POSITIVE_45;
                    break;
                case POSITIVE_45:
                    moveToPosition(45.0, "Moving to +45°");
                    phase = TestPhase.POSITIVE_90;
                    break;
                case POSITIVE_90:
                    moveToPosition(90.0, "Moving to +90° (max)");
                    phase = TestPhase.RETURN_TO_ZERO;
                    break;
                case RETURN_TO_ZERO:
                    moveToPosition(0.0, "Returning to 0°");
                    phase = TestPhase.NEGATIVE_45;
                    break;
                case NEGATIVE_45:
                    moveToPosition(-45.0, "Moving to -45°");
                    phase = TestPhase.NEGATIVE_90;
                    break;
                case NEGATIVE_90:
                    moveToPosition(-90.0, "Moving to -90° (min)");
                    phase = TestPhase.FINAL_ZERO;
                    break;
                case FINAL_ZERO:
                    moveToPosition(0.0, "Returning to 0° (final)");
                    phase = TestPhase.COMPLETE;
                    break;
                case COMPLETE:
                    // Test finished
                    break;
            }
        }
        
        // Update telemetry
        SmartDashboard.putString("Turret Test Phase", phase.toString());
    }
    
    /**
     * Move both turrets to a target position.
     */
    private void moveToPosition(double angleDegrees, String message) {
        RobotContainer.turretLeft.setTargetAngle(angleDegrees);
        RobotContainer.turretRight.setTargetAngle(angleDegrees);
        SmartDashboard.putString("Turret Test Phase", message);
        System.out.println(message);
    }
    
    /**
     * Log the current turret positions.
     */
    private void logPosition() {
        double leftAngle = RobotContainer.turretLeft.getCurrentAngle();
        double rightAngle = RobotContainer.turretRight.getCurrentAngle();
        double leftTarget = RobotContainer.turretLeft.getTargetAngle();
        double rightTarget = RobotContainer.turretRight.getTargetAngle();
        
        System.out.println(String.format(
            "Phase: %s | Left: %.2f° (target %.2f°) | Right: %.2f° (target %.2f°)",
            phase, leftAngle, leftTarget, rightAngle, rightTarget
        ));
        
        SmartDashboard.putNumber("Test Left Angle", leftAngle);
        SmartDashboard.putNumber("Test Right Angle", rightAngle);
    }
    
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("=== TURRET CALIBRATION TEST INTERRUPTED ===");
        } else {
            System.out.println("=== TURRET CALIBRATION TEST COMPLETE ===");
            System.out.println("Review encoder values above to verify:");
            System.out.println("1. Zero position is mechanically correct");
            System.out.println("2. Soft limits prevent over-rotation");
            System.out.println("3. Encoder values are accurate at known angles");
        }
        
        // Return to zero
        RobotContainer.turretLeft.setTargetAngle(0.0);
        RobotContainer.turretRight.setTargetAngle(0.0);
        
        SmartDashboard.putString("Turret Test Phase", interrupted ? "Interrupted" : "Complete");
    }
    
    @Override
    public boolean isFinished() {
        return phase == TestPhase.COMPLETE;
    }
}
