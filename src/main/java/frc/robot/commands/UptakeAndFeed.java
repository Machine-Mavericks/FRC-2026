package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperFeed;
import frc.robot.subsystems.Uptake;


// command template
public class UptakeAndFeed extends Command {
 private HopperFeed hopperFeed;
  private Uptake uptake;

  private long unjamTimeout = 0;

    // constructor
    public UptakeAndFeed(HopperFeed hopperFeed, Uptake uptake) {

        this.hopperFeed = hopperFeed;
        addRequirements(hopperFeed);
        
        this.uptake = uptake;
        addRequirements(uptake);
      // add subsystem requirements (if any) - for example:
        //addRequirements(RobotContainer.drivesystem);
    }

    // This method is called once when command is started
    @Override
    public void initialize() {

    }

    // This method is called periodically while command is active
    @Override
    public void execute() {
        hopperFeed.feed();
        uptake.feedShooter();
    }

    // This method to return true only when command is to finish. Otherwise return false
    @Override
    public boolean isFinished() {

        return false;

    }

    // This method is called once when command is finished.
    @Override
    public void end(boolean interrupted) {
        hopperFeed.stop();
        uptake.stop();
    }

}