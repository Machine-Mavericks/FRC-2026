// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import frc.robot.tests.HardwareTestSuite;
//import frc.robot.commands.TiltAlgaeRemover;

//import frc.robot.utils.AlgaePositions;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final SendableChooser<String> m_testChooser = new SendableChooser<>();
  private double m_armTestTarget = 0.0;
  // private HardwareTestSuite m_testSuite;

  // private boolean isAlgaeTiltInitialized = false;
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer(this);

    m_testChooser.setDefaultOption("IntakeArm", "IntakeArm");
    m_testChooser.addOption("Turrets", "Turrets");
    m_testChooser.addOption("Shooter", "Shooter");
    m_testChooser.addOption("Uptake", "Uptake");
    m_testChooser.addOption("IntakeSubsystem", "IntakeSubsystem");
    SmartDashboard.putData("Test Subsystem", m_testChooser);

    // Build the hardware test suite (creates the "Hardware Tests" Shuffleboard
    // tab).
    // m_testSuite = new HardwareTestSuite();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /// ** This autonomous runs the autonomous command selected by your {@link
  /// RobotContainer} class. */
  @Override
  public void autonomousInit() {

    // Get and schedule the autonomous command
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // if (m_autonomousCommand != null) {
    // edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().schedule(m_autonomousCommand);
    // }

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // RobotContainer.odometry.EnableApriltagProcessing(true);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    // Initialize the test target to the current position to prevent sudden snaps on enable
    m_armTestTarget = RobotContainer.intakeArm.leftCurrentPose;
  }

  /**
   * Called periodically in Test mode.
   * Press the OPERATOR START button to trigger the full hardware smoke-test
   * sequence.
   * Watch the "Hardware Tests" Shuffleboard tab for PASS / FAIL results.
   */
  @Override
  public void testPeriodic() {
    String testSubsystem = m_testChooser.getSelected();
    if (testSubsystem == null) return;

   // RobotContainer.intakeArm.debug(0.1);

    
    switch (testSubsystem) {
      case "IntakeArm":
        // --- IntakeArm Tuning Controls ---
        // 1. Position Setup (Use these to synchronize the encoder before tuning)
        if (RobotContainer.toolOp.back().getAsBoolean()) {
          RobotContainer.intakeArm.zeroEncoder();
          System.out.println("Test Mode: IntakeArm encoders zeroed (Horizontal)");
        }
        if (RobotContainer.toolOp.start().getAsBoolean()) {
          RobotContainer.intakeArm.setStowedPosition();
          System.out.println("Test Mode: IntakeArm encoders set to STOWED (-90 deg)");
        }

        // 2. Motion Magic Testing (Use these to test kG and kP)
        if (RobotContainer.toolOp.getHID().getPOV() == 0) { // D-Pad Up
          m_armTestTarget = RobotMap.IntakeArm.STOWED_POSITION;
          System.out.println("test mode: target set to STOWED");
        } else if (RobotContainer.toolOp.getHID().getPOV() == 180) { // D-Pad Down
          m_armTestTarget = RobotMap.IntakeArm.DEPLOYED_POSITION;
          System.out.println("test mode: target set to DEPLOYED");
        }

        // Use snapTo instead of moveTo for testing to allow full Motion Magic profiles
        // (bypasses the 5-degree 'leash' logic)
        RobotContainer.intakeArm.snapTo(m_armTestTarget);
        break;

      case "Turrets":
        // --- Turret Tuning Controls ---
        // 1. Position Setup (Zero encoders when pointing straight ahead)
        if (RobotContainer.toolOp.leftBumper().getAsBoolean()) {
          RobotContainer.turretLeft.resetEncoder();// should do nothing 
          RobotContainer.turretRight.resetEncoder();// should do somthing
          System.out.println("Test Mode: Turret encoders zeroed (Straight Ahead)");
        }

        // 2. PID Tracking Testing (Snap to angles)
        if (RobotContainer.toolOp.a().getAsBoolean()) {
          RobotContainer.turretLeft.setTargetAngle(0.0);// should do nothing 
          RobotContainer.turretRight.setTargetAngle(0.0);// should do somthing
        } else if (RobotContainer.toolOp.getHID().getPOV() == 270) { // D-Pad Left
          RobotContainer.turretLeft.setTargetAngle(RobotMap.Turret.MIN_ROTATION_DEGREES);
          RobotContainer.turretRight.setTargetAngle(RobotMap.Turret.MIN_ROTATION_DEGREES);
        } else if (RobotContainer.toolOp.getHID().getPOV() == 90) { // D-Pad Right
          RobotContainer.turretLeft.setTargetAngle(RobotMap.Turret.MAX_ROTATION_DEGREES);
          RobotContainer.turretRight.setTargetAngle(RobotMap.Turret.MAX_ROTATION_DEGREES);
        }
        break;

      case "Shooter":
        // --- Shooter Tuning Controls ---
        if (RobotContainer.toolOp.y().getAsBoolean()) {
          RobotContainer.shooter.shooterSpeed(60.0); // Test target speed: 60 RPS
        } else {
          RobotContainer.shooter.stop();
        }
        break;

      case "Uptake":
        // --- Uptake Tuning Controls ---
        if (RobotContainer.toolOp.rightBumper().getAsBoolean()) {
          RobotContainer.uptake.feedShooter();
        } else {
          RobotContainer.uptake.stop();
        }
        break;

      case "IntakeSubsystem":
        // --- IntakeSubsystem Tuning Controls ---
        if (RobotContainer.toolOp.leftBumper().getAsBoolean()) {
          RobotContainer.intake.intake();
        } else if (RobotContainer.toolOp.rightBumper().getAsBoolean()) {
          RobotContainer.intake.outtake();
        } else {
          RobotContainer.intake.stop();
        }
        break;

      default:
        break;
    }
         
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
