package frc.robot;

import frc.robot.commands.IncrementShootersSpeed;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.MoveToPose;
import frc.robot.commands.ParkCommand;
import frc.robot.commands.Pause;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TemplateCommand;
import frc.robot.commands.UptakeAndFeed;
import frc.robot.commandgroups.DepotAuto;
import frc.robot.commandgroups.JogUptakeAndFeedCommand;
import frc.robot.commandgroups.MoveOffLineAndShootPreloadsAuto;
import frc.robot.commandgroups.ShootPreloadsAuto;
import frc.robot.commandgroups.StraightAuto;
import frc.robot.commandgroups.SweepAuto;
import frc.robot.commands.JogIntakeArm;
import frc.robot.commands.AutoTrackGoal;
import frc.robot.commands.FixOdometry;
import frc.robot.commands.HopperJogBack;
import frc.robot.commands.ManualTurretControl;
import frc.robot.subsystems.HubTargetingSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.MainShuffleBoardTab;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.TemplateSubsystem;
import frc.robot.subsystems.TurretLeft;
import frc.robot.subsystems.TurretLeft;
import frc.robot.subsystems.TurretRight;
import frc.robot.subsystems.TurretDisabled;
import frc.robot.subsystems.ShooterDisabled;
import frc.robot.subsystems.UptakeDisabled;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.HopperFeed;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.Uptake;
import frc.robot.commands.IntakeSequence;
import frc.robot.commands.ShootSequence;
import frc.robot.utils.AutoFunctions;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    public static Limelight limelightDrive;
    public static Limelight limelightShooter;
    public static HubTargetingSubsystem hubTargeting;
    public static Pigeon gyro;
    public static SwerveDrive drivesystem;
    public static Odometry odometry;
    public static TemplateSubsystem mySubsystem;

    // define shooter subsystem
    // define shooter subsystem
    public static Shooter leftShooter;
    public static Shooter rightShooter;
    public static TurretLeft turretLeft;
    public static TurretRight turretRight;
    public static IntakeSubsystem intake;
    public static IntakeArm intakeArm;
    public static Uptake uptake;
    public static HopperFeed hopperFeed;

    // AutoTrackGoal is the default command on both turrets; exposed as a static
    // field so isReadyToShoot() and getCalculatedShooterRPM() are reachable from
    // anywhere (e.g. button bindings). Fixes issue #13.
    public static AutoTrackGoal autoTrack;

    // main shuffleboar page
    public static MainShuffleBoardTab mainShufflePage = new MainShuffleBoardTab();

    public static PowerDistribution pdh;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(Robot robotptr) {

        // record pointer to robot object
        robot = robotptr;

        // create driver(port 0) and operator(port 1) controllers
        driverOp = new CommandXboxController(RobotMap.GamePadPorts.DriverID);
        toolOp = new CommandXboxController(RobotMap.GamePadPorts.OperatorID);

        // create instances of subsystems here
        gyro = new Pigeon();

        limelightDrive = new Limelight(RobotMap.Vision.LIMELIGHT_DRIVE_NAME);
        limelightDrive.setCameraPoseRobotSpace(RobotMap.Vision.DRIVE_LIMELIGHT_3D_POSE);
        // Create Limelight for AprilTag detection — filter to all HUB tags (both
        // alliances)
        limelightShooter = new Limelight(RobotMap.Vision.LIMELIGHT_SHOOTER_NAME);
        limelightShooter.setCameraPoseRobotSpace(RobotMap.Vision.SHOOTER_LIMELIGHT_3D_POSE);
        int[] allHubTagIds = concatArrays(RobotMap.Vision.BLUE_HUB_TAG_IDS, RobotMap.Vision.RED_HUB_TAG_IDS);
        limelightShooter.SetFiducialIDFiltersOverride(allHubTagIds);

        // Create hub targeting subsystem (reads from limelight, computes distance +
        // RPM)
        hubTargeting = new HubTargetingSubsystem();

        drivesystem = new SwerveDrive();
        drivesystem.setDefaultCommand(new ManualDrive());
        odometry = new Odometry();
        mySubsystem = new TemplateSubsystem();

        // Create shooter, turret, and uptake subsystems.
        // If left turret feature is disabled, instantiate no-op stubs so the
        // rest of the code (commands, default commands) can run without
        // null-checks or hardware present.
        leftShooter = new Shooter(RobotMap.CANID.LEFT_SHOOTER, InvertedValue.Clockwise_Positive);
        rightShooter = new Shooter(RobotMap.CANID.RIGHT_SHOOTER, InvertedValue.CounterClockwise_Positive);

        if (frc.robot.RobotMap.Features.ENABLE_LEFT_TURRET) {
            turretLeft = new TurretLeft();
        } else {
            // Left turret hardware missing -> provide software stubs
            turretLeft = new TurretDisabled();
        }

        uptake = new Uptake(false);

        // Right turret (present) and other subsystems
        turretRight = new TurretRight();

        // Create intake subsystems
        intake = new IntakeSubsystem();
        intakeArm = new IntakeArm();
        hopperFeed = new HopperFeed();

        // Set default command for turrets (auto-tracking)
        autoTrack = new AutoTrackGoal();
        turretLeft.setDefaultCommand(autoTrack);
        turretRight.setDefaultCommand(autoTrack);

        pdh = new PowerDistribution(1, ModuleType.kRev);

        SmartDashboard.putData("Power Distribution", pdh);

        // attach commands to controller buttons
        configureBindings();
    }

    /** Use this method to define your trigger->command mappings. */
    private void configureBindings() {

        // attach commands to buttons

        // reset odometry to appropriate angle when back pressed.
        driverOp.back().onTrue(new InstantCommand(() -> {
            Pose2d pos = odometry.getPose2d();
            Rotation2d newHeading = AutoFunctions.redVsBlue(new Rotation2d(0.0));
            odometry.setPose(0.0, 0.0, newHeading.getRadians(), newHeading.getRadians());
        }));

        driverOp.start().onTrue(new FixOdometry());

        driverOp.leftBumper().whileTrue(new ParkCommand(drivesystem));

        // operator controls

        // Manual turret control - hold left bumper to override auto-tracking
        // toolOp.leftBumper().whileTrue(new ManualTurretControl());

        // description of commands available:
        // .onTrue - runs command when button changes from not-pressed to pressed.
        // .onFalse - runs command when button changes from pressed to not-pressed.
        // .onChange - runs command when state changes either way
        // .whileTrue - runs command only while button is pressed - command does not
        // restart if finished
        // .whileFalse - runs command only while button is not pressed - command does
        // not restart if finished

        // to have command automatically repeat if it finishes while button is pressed
        // or whatever
        // toolOp.back().whileTrue(new RepeatCommand(new TemplateCommand()));

        // to debounce the trigger event
        // driverOp.y().debounce(0.5).onTrue(new TemplateCommand());

        // to use a trigger as a button - note: analog triggers should be debounced as
        // well
        // driverOp.rightTrigger(0.5).debounce(0.25).onTrue(new TemplateCommand());

        // Operator controlling shooter speeds
        // toolOp.b().onTrue(new IncrementShootersSpeed(rightShooter, 1.0));
        // toolOp.a().onTrue(new IncrementShootersSpeed(shooter, -0.5));
        // toolOp.y().onTrue(new IncrementShootersSpeed(shooter, 5.0));
        // toolOp.x().onTrue(new IncrementShootersSpeed(rightShooter, -1.0));

        // toolOp.a().whileTrue(new
        // InstantCommand(()->shooter.shooterSpeed(shooter.CalculateSpeed())));
        // toolOp.a().onFalse(new InstantCommand(()->rightShooter.shooterSpeed(0.0)));

        // toolOp.b().onTrue(new InstantCommand(()->shooter.shooterSpeed(58.7)));
        // Fire using the new automated sequence
        // Uses the RPM computed by HubTargetingSubsystem each loop (issue #15).
        // toolOp.rightTrigger().whileTrue(new ShootSequence(shooter, intakeArm,
        // uptake)); // test next

        // Intake control - toggle right bumper: first press deploys arm + runs intake,
        // second press stops intake and stows arm.
        // toolOp.rightBumper().toggleOnTrue(new IntakeSequence(intake, intakeArm)); //
        // test next
        toolOp.rightBumper().whileTrue(new JogUptakeAndFeedCommand(hopperFeed, uptake));
        toolOp.leftBumper().whileTrue(new RunIntakeCommand(intake));
        // toolOp.leftTrigger().onTrue(new InstantCommand(()->intakeArm.moveTo(-5.0 /
        // 360.0)));
        // toolOp.rightTrigger().onTrue(new InstantCommand(()->intakeArm.moveTo(-90.0 /
        // 360.0)));
        toolOp.y().whileTrue(new ShootCommand(leftShooter, rightShooter));
        toolOp.back().whileTrue(new HopperJogBack(hopperFeed));

        toolOp.povDown().whileTrue(new JogIntakeArm(-0.15));
        toolOp.povUp().whileTrue(new JogIntakeArm(0.15));
    }

    public Command getAutonomousCommand() {

        // get autonomous path to run
        // for example, a subsystem could made responsible for returning selected path
        // from a list populated in shuffleboard.
        int index = RobotContainer.mainShufflePage.getSelectedAutoIndex(); // RobotContainer.autopathselect.GetSelectedPath();

        Command chosenCommand = null;
        System.out.println("Getting Autonomous Command!");
        System.out.println(index);

        // return autonomous command to be run in autonomous
        if (index == 0)
            chosenCommand = new Pause(20.0); // do nothing command
        else if (index == 1)
            chosenCommand = new ShootPreloadsAuto(true); // drive off the line
        else if (index == 2)
            chosenCommand = new ShootPreloadsAuto(false);
        else if (index == 3)
            chosenCommand = new DepotAuto();
   
        //     else if (index == 3)
        //     chosenCommand = new OneCoralAutoCenter();
        //     else if (index == 4)
        //     chosenCommand = new OneCoralAutoLeft();
        //     else if (index == 5)
        //     chosenCommand = new TwoCoralAutoLeft();
        

        return new SequentialCommandGroup(
                new Pause(RobotContainer.mainShufflePage.getAutoDelay()),
                chosenCommand);
    }

    /**
     * Helper: concatenate two int arrays into one (used to combine blue + red HUB
     * tag IDs).
     */
    private static int[] concatArrays(int[] a, int[] b) {
        int[] result = new int[a.length + b.length];
        System.arraycopy(a, 0, result, 0, a.length);
        System.arraycopy(b, 0, result, a.length, b.length);
        return result;
    }
}
