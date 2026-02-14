package frc.robot;


import frc.robot.commands.ManualDrive;
import frc.robot.commands.MoveToPose;
import frc.robot.commands.Pause;
import frc.robot.commands.TemplateCommand;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.TemplateSubsystem;
import frc.robot.subsystems.Uptake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Odometry;
import frc.robot.utils.AutoFunctions;
import frc.robot.utils.ElevatorPositions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.robot.utils.AlgaePositions;


public class RobotContainer {
  
    // pointer to robot object
    public static Robot robot;

    // create driver and operator xBox controllers
    public static CommandXboxController driverOp;
    public static CommandXboxController toolOp;

    
    // make pointers to robot subsystems here
    public static Pigeon gyro;
    public static SwerveDrive drivesystem;
    public static Odometry odometry;
    public static Uptake uptake;
    public static Shooter shooter;
    public static TemplateSubsystem mySubsystem;
    public static Climber climber;
    public static Hopper hopper;
    public static Intake intake;
   
   
   
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer(Robot robotptr) {

        // record pointer to robot object
        robot = robotptr;

        // create driver(port 0) and operator(port 1) controllers
        driverOp = new CommandXboxController(RobotMap.GamePadPorts.DriverID);
        toolOp = new CommandXboxController(RobotMap.GamePadPorts.OperatorID);
    

        // create instances of subsystems here
        gyro = new Pigeon();
        drivesystem = new SwerveDrive();
        drivesystem.setDefaultCommand(new ManualDrive());
        odometry = new Odometry();
        uptake = new Uptake();
        shooter = new Shooter();
        mySubsystem = new TemplateSubsystem();
        climber = new Climber();
        hopper = new Hopper();
        intake = new Intake();
       
        
       
        // attach commands to controller buttons
        configureBindings();
    }

   
    /** Use this method to define your trigger->command mappings. */
    private void configureBindings() {
    
        // attach commands to buttons
    
        // reset odometry to appropriate angle when back pressed.
        driverOp.back().onTrue(new InstantCommand(()-> {
            Pose2d pos = odometry.getPose2d();
            Rotation2d newHeading = AutoFunctions.redVsBlue(new Rotation2d(0.0));
            odometry.setPose(0.0, 0.0, newHeading.getRadians(), newHeading.getRadians());
        } ));

         driverOp.a().onTrue(new InstantCommand(()-> {
           intake.spin(0.5);
           
        } ));

         driverOp.b().onTrue(new InstantCommand(()-> {
           intake.spin(0.0);
           
        } ));
        

        // operator controls 
       
        
        // description of commands available:
        // .onTrue - runs command when button changes from not-pressed to pressed.
        // .onFalse - runs command when button changes from pressed to not-pressed.
        // .onChange - runs command when state changes either way
        // .whileTrue - runs command only while button is pressed - command does not restart if finished
        // .whileFalse - runs command only while button is not pressed - command does not restart if finished

        // to have command automatically repeat if it finishes while button is pressed or whatever
        // toolOp.back().whileTrue(new RepeatCommand(new TemplateCommand()));

        // to debounce the trigger event
        // driverOp.y().debounce(0.5).onTrue(new TemplateCommand());

        // to use a trigger as a button - note: analog triggers should be debounced as well
        // driverOp.rightTrigger(0.5).debounce(0.25).onTrue(new TemplateCommand());
    }


    /** Use this function to return pointer to the command the robot is to follow in autonomous
    * @return the command to run in autonomous */
       public Command getAutonomousCommand() {
           // Example: Move robot forward 1 meter, no rotation
           return new frc.robot.commands.MoveRobotRelative(
               1.0, // maxSpeed
               0.5, // maxAccel (used as maxRotSpeed)
               new Pose2d(.5, 0.0, new Rotation2d(0.0)) // move 1 meter forward
           );
       }
    }
