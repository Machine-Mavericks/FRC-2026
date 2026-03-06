package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

//TODO add java docs
public class IncrementShootersSpeed extends Command {
    
    private final Shooter shooter;

    private double newSpeed;

    /**
     * Sets the speed of the shooter in RPS
     * 
     * @param subsystem The shooter subsystem
     * @param newSpeed The desired speed of the shooter in RPS
     */
    public IncrementShootersSpeed(Shooter shooter, double newSpeed) {
        this.shooter = shooter;
        this.newSpeed = newSpeed;

        addRequirements(shooter);
    }

    // Called when the command is inititally scheduled
    @Override
    public void initialize() {}

    // Called every time the schedular runs while the command is scheduled
    @Override
    public void execute() {
        double desired = shooter.velocity + newSpeed;
        double newVal = Math.min(Math.max(desired, -80), 80);

        shooter.shooterSpeed(newVal);
    }

    // Called once the command ends or is interrupted
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end
    @Override
    public boolean isFinished() {
        return true;
    }
}
