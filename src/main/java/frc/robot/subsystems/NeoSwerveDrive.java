// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.RobotMap.AINPorts;
import frc.robot.utils.Utils;

/** Subsystem */
public class NeoSwerveDrive extends SubsystemBase {

    // Setup Drive Kinematics Object Constants
    private final double TRACK_WIDTH = 0.31;   // Width between the left and right wheels - in m.
    private final double TRACK_LENGTH = 0.29;  // Length between the front and back wheel - in m.

    // steering gear ratio for SPS MK1 Swerve
    private static final double STEER_RATIO = 18.0;

    // drive gear ratio for SPS MK1 Swerve
    private static final double DRIVE_RATIO = 8.33;

    // drive wheel diameter in m  (4" VexPro 217-3197)
    private final double WHEEL_DIA = 0.1016;   // new with thread = 0.1032002
                                               // 4"=0.1016
    
    // conversion factors - used to convert motor MPS to RPS and vice versa
    private final double WHEELRPS_TO_MPS = Math.PI * WHEEL_DIA;
    private final double MPS_TO_WHEELRPS = 1.0 /  WHEELRPS_TO_MPS;
    private final double WHEELRPM_TO_MPS = (1.0/60.0)*WHEELRPS_TO_MPS;
    private final double MPS_TO_WHEELRPM = 1.0 / WHEELRPM_TO_MPS;


    // constants for Rev Neo motors
    private final double MAXRPM = 5676.0 * 0.975;
    private final double MAXRPS = MAXRPM / 60.0;

    // maximum expected drive speed or robot (m/s)
    public final double MAX_SPEED = MAXRPS * WHEELRPS_TO_MPS / DRIVE_RATIO;


    // The drivetrain's kinematics model
    private SwerveDriveKinematics driveKinematics;
    private final Translation2d m_CenterOfRotation = new Translation2d(0.0,0.0);;
    private final Rotation2d m_ParkAngleLF = new Rotation2d(45.0);
    private final Rotation2d m_ParkAngleRF = new Rotation2d(-45.0);

    // create steering angle sensor objects
    private AnalogPotentiometer m_LFAngleSensor;
    private AnalogPotentiometer m_RFAngleSensor;
    private AnalogPotentiometer m_LRAngleSensor;
    private AnalogPotentiometer m_RRAngleSensor;

    // create steer motor objects
    private SparkMax m_LFSteerMotor;
    private SparkMax m_RFSteerMotor;
    private SparkMax m_LRSteerMotor;
    private SparkMax m_RRSteerMotor;

    // steer motor closed loop target references
    private double m_LFSteerMotor_Ref;
    private double m_RFSteerMotor_Ref;
    private double m_LRSteerMotor_Ref;
    private double m_RRSteerMotor_Ref;

    // create drive motor objects
    private SparkMax m_LFDriveMotor;
    private SparkMax m_RFDriveMotor;
    private SparkMax m_LRDriveMotor;
    private SparkMax m_RRDriveMotor;

    // drive motor closed loop target references
    private double m_LFDriveMotor_Ref;
    private double m_RFDriveMotor_Ref;
    private double m_LRDriveMotor_Ref;
    private double m_RRDriveMotor_Ref;



    // Swerve module states - contains target speed(m/s) and angle for each swerve module
    private SwerveModuleState[] m_states;


    /** Place code here to initialize subsystem */
    public NeoSwerveDrive() {

        // setup the drive Kinematics
        driveKinematics = new SwerveDriveKinematics(
                // Front Left
                new Translation2d(TRACK_LENGTH * 0.5, TRACK_WIDTH * 0.5),
                // Front Right
                new Translation2d(TRACK_LENGTH * 0.5, TRACK_WIDTH * -0.5),
                // Back Left
                new Translation2d(TRACK_LENGTH * -0.5, TRACK_WIDTH * 0.5),
                // Back Right
                new Translation2d(TRACK_LENGTH * -0.5, TRACK_WIDTH * -0.5));

        
        // ---------- Steer Angle Sensor Setup ----------

        // steering sensor offsets - ADJUST values to align wheels
        // units - in degrees
        // note:  if +ve value when wheels are aligned straight, then decrease offset by that value
        // encoders should then read zero when aligned straight.
        // when aligning, wheel ring gears face inside robot
        double LF_STEER_OFFSET = -177.75/360.0; //4.2;  -177.6
        double RF_STEER_OFFSET = 1.09/360.0; //96.0;  //0.45
        double LR_STEER_OFFSET = -95.5/360.0; //0.3; //-96.1
        double RR_STEER_OFFSET = 7.9/360.0; //178.2; //11.8
        
        // create analog inputs, set oversampling and average
        // then associate analog inputs with each position sensor
        AnalogInput input;
        input = new AnalogInput(AINPorts.NEO_LF_STEER_SENSOR);
        input.setOversampleBits(4); input.setAverageBits(512);
        m_LFAngleSensor = new AnalogPotentiometer(input, 1.0, LF_STEER_OFFSET-0.5);

        input = new AnalogInput(AINPorts.NEO_RF_STEER_SENSOR);
        input.setOversampleBits(4); input.setAverageBits(512);
        m_RFAngleSensor = new AnalogPotentiometer(input, 1.0, RF_STEER_OFFSET-0.5);
    
        input = new AnalogInput(AINPorts.NEO_LR_STEER_SENSOR);
        input.setOversampleBits(4); input.setAverageBits(512);
        m_LRAngleSensor = new AnalogPotentiometer(input, 1.0, LR_STEER_OFFSET-0.5);

        input = new AnalogInput(AINPorts.NEO_RR_STEER_SENSOR);
        input.setOversampleBits(4); input.setAverageBits(512);
        m_RRAngleSensor = new AnalogPotentiometer(input, 1.0, RR_STEER_OFFSET-0.5);

        
        // ---------- Steer Motor Setup ----------

        // create steer motors
        m_LFSteerMotor = new SparkMax(RobotMap.CANID.LF_STEER_MOTOR, MotorType.kBrushless);
        m_RFSteerMotor = new SparkMax(RobotMap.CANID.RF_STEER_MOTOR, MotorType.kBrushless);
        m_LRSteerMotor = new SparkMax(RobotMap.CANID.LR_STEER_MOTOR, MotorType.kBrushless);
        m_RRSteerMotor = new SparkMax(RobotMap.CANID.RR_STEER_MOTOR, MotorType.kBrushless);
       
        // configure steer motors
        SparkMaxConfig steerConfig = new SparkMaxConfig();
        steerConfig.inverted(true);
        steerConfig.idleMode(IdleMode.kCoast);
        steerConfig.inverted(false);
        steerConfig.closedLoop.p(10.0);
        steerConfig.closedLoop.i(0.2);
        steerConfig.closedLoop.d(0.0);
        steerConfig.closedLoop.outputRange(-1.0, 1.0);
        steerConfig.closedLoop.positionWrappingEnabled(true);
        steerConfig.encoder.positionConversionFactor(1.0 / STEER_RATIO);
        // deadband ??

        m_LFSteerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_RFSteerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_LRSteerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_RRSteerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    
        
        // ---------- Drive Motor Setup ----------

        // create drive motors
        m_LFDriveMotor = new SparkMax(RobotMap.CANID.LF_DRIVE_MOTOR, MotorType.kBrushless);
        m_RFDriveMotor = new SparkMax(RobotMap.CANID.RF_DRIVE_MOTOR, MotorType.kBrushless);
        m_LRDriveMotor = new SparkMax(RobotMap.CANID.LR_DRIVE_MOTOR, MotorType.kBrushless);
        m_RRDriveMotor = new SparkMax(RobotMap.CANID.RR_DRIVE_MOTOR, MotorType.kBrushless);

        // configure drive motors
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.idleMode(IdleMode.kCoast);
        driveConfig.closedLoop.p(0.0005);
        driveConfig.closedLoop.i(0.000008);
        driveConfig.closedLoop.d(0.0);
        driveConfig.closedLoop.iMaxAccum(0.2);
        driveConfig.closedLoop.velocityFF(0.0012);
        driveConfig.closedLoop.outputRange(-1.0, 1.0);
        driveConfig.encoder.velocityConversionFactor(1.0 / DRIVE_RATIO);
        driveConfig.encoder.positionConversionFactor(1.0 / DRIVE_RATIO);
        //iMaxAccum??
        // deadband ??

        driveConfig.inverted(false);
        m_LFDriveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveConfig.inverted(true);
        m_RFDriveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveConfig.inverted(false);
        m_LRDriveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveConfig.inverted(true);
        m_RRDriveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        
         // ---------- Other ----------

        
        // initialize encoders of each steer motor according to CANCoder positions
        ResetSteerEncoders();

        // create subsystem shuffle board page
        initializeShuffleboard();

        // initially motors are off
        RobotDrive(0.0, 0.0, 0.0, false);
    }

    // seed the encoder value of steering motors based on reported CANcoder positions
    public void ResetSteerEncoders() {
    
        // wheel alignment calibration robot base
        m_LFSteerMotor.getEncoder().setPosition(-m_LFAngleSensor.get());
        m_RFSteerMotor.getEncoder().setPosition(-m_RFAngleSensor.get());
        m_LRSteerMotor.getEncoder().setPosition(-m_LRAngleSensor.get());
        m_RRSteerMotor.getEncoder().setPosition(-m_RRAngleSensor.get());
    }


    /** Method called periodically by the scheduler */
    @Override
    public void periodic() {

        // update shuffle board values
        updateShuffleboard(); 

    }

    /** Drive robot in field-oriented coordinates
      ChassisSpeeds x,y in m/s, omega in rad/s
      Park - places robot in park mode - other inputs are ignored */
    public void FieldDrive(double dx, double dy, double omega, boolean Park)
        { FieldDrive (new ChassisSpeeds(dx, dy, omega), Park); }

    public void FieldDrive(ChassisSpeeds speed, boolean Park) {
        // convert chassis speeds from field to robot oriented, getting angle from gyro
        ChassisSpeeds newSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(speed, Rotation2d.fromDegrees(RobotContainer.gyro.getYawAngle()));
    
        // speeds now in robot coordinates - call robot drive
        RobotDrive (newSpeed, Park);
    }

    /** Drive robot in robot-oriented coordinates
      ChassisSpeeds x,y in m/s, omega in rad/s
      Park - places robot in park mode - other inputs are ignored */
    public void RobotDrive(double dx, double dy, double omega, boolean Park)
        { RobotDrive (new ChassisSpeeds(dx, dy, omega), Park); }
    
    public void RobotDrive(ChassisSpeeds speed, boolean Park) {
        
        // get desired swerve states from chassis speeds using defined kinematics
        m_states = driveKinematics.toSwerveModuleStates(speed, m_CenterOfRotation); 

        // Park will override any of the drive inputs for chassis speeds
        if (Park) {
            // Note that the LF and RR wheels are both set to the same angle. Same for the RF and LR wheels
            m_states[0].angle = m_ParkAngleLF;       m_states[0].speedMetersPerSecond = 0;
            m_states[1].angle = m_ParkAngleRF;       m_states[1].speedMetersPerSecond = 0;
            m_states[2].angle = m_ParkAngleRF;       m_states[2].speedMetersPerSecond = 0;
            m_states[3].angle = m_ParkAngleLF;       m_states[3].speedMetersPerSecond = 0;
        }

        // if desired speed of a swerve(s) exceed maximum possible, then reduce all speeds while maintaining ratio
        SwerveDriveKinematics.desaturateWheelSpeeds(m_states, MAX_SPEED);


        // assume drive motors driving in forward direction until determined otherwise
        double LFDriveDir = 1.0;
        double RFDriveDir = 1.0;
        double LRDriveDir = 1.0;
        double RRDriveDir = 1.0;
        
        // ---------- Angle Determination for LF Swerve

        // get steering motor position - convert from rotations to deg
        double LFCurrentAngleDeg = m_LFSteerMotor.getEncoder().getPosition()*360.0;

        // determine target angle in degrees
        double LFTargetAngleDeg = m_states[0].angle.getDegrees();

        // determine smallest angle to turn swerve to get to desired angle
        double LFAngleDiff = Utils.AngleDifference(LFCurrentAngleDeg, LFTargetAngleDeg);
        
        // To minimize turning, if we need to turn more than 90deg,
        // it may be easier to reverse drive and turn by smaller angle
        if (LFAngleDiff<-90.0)
            { LFDriveDir *= -1.0; LFAngleDiff+=180.0; }
        else if (LFAngleDiff>90)
            { LFDriveDir *= -1.0; LFAngleDiff-=180.0; }

        // set angle of swerve drive (convert deg back to rotations = 1/360.0)
        m_LFSteerMotor_Ref = (LFCurrentAngleDeg + LFAngleDiff)*0.00277777;
        m_LFSteerMotor.getClosedLoopController().setReference(m_LFSteerMotor_Ref,ControlType.kPosition, ClosedLoopSlot.kSlot0);
        

        // ---------- Angle Determination for RF Swerve

        
        double RFCurrentAngleDeg = m_RFSteerMotor.getEncoder().getPosition()*360.0;
        double RFTargetAngleDeg = m_states[1].angle.getDegrees();
        double RFAngleDiff = Utils.AngleDifference(RFCurrentAngleDeg, RFTargetAngleDeg);
        if (RFAngleDiff<-90.0)
            { RFDriveDir *= -1.0; RFAngleDiff+=180.0; }
        else if (RFAngleDiff>90)
            { RFDriveDir *= -1.0; RFAngleDiff-=180.0; }
        m_RFSteerMotor_Ref = (RFCurrentAngleDeg + RFAngleDiff)*0.00277777;
        m_RFSteerMotor.getClosedLoopController().setReference(m_RFSteerMotor_Ref,ControlType.kPosition, ClosedLoopSlot.kSlot0);

        // ---------- Angle Determination for LR Swerve

        
        double LRCurrentAngleDeg = m_LRSteerMotor.getEncoder().getPosition()*360.0;
        double LRTargetAngleDeg = m_states[2].angle.getDegrees();
        double LRAngleDiff = Utils.AngleDifference(LRCurrentAngleDeg, LRTargetAngleDeg);
        if (LRAngleDiff<-90.0)
            { LRDriveDir *= -1.0; LRAngleDiff+=180.0; }
        else if (LRAngleDiff>90)
            { LRDriveDir *= -1.0; LRAngleDiff-=180.0; }
        m_LRSteerMotor_Ref = (LRCurrentAngleDeg + LRAngleDiff)*0.00277777;
        m_LRSteerMotor.getClosedLoopController().setReference(m_LRSteerMotor_Ref,ControlType.kPosition, ClosedLoopSlot.kSlot0);

        
        // ---------- Angle Determination for RR Swerve

        double RRCurrentAngleDeg = m_RRSteerMotor.getEncoder().getPosition()*360.0;
        double RRTargetAngleDeg = m_states[3].angle.getDegrees();
        double RRAngleDiff = Utils.AngleDifference(RRCurrentAngleDeg, RRTargetAngleDeg);
        if (RRAngleDiff<-90.0)
            { RRDriveDir *= -1.0; RRAngleDiff+=180.0; }
        else if (RRAngleDiff>90)
            { RRDriveDir *= -1.0; RRAngleDiff-=180.0; }
        m_RRSteerMotor_Ref = (RRCurrentAngleDeg + RRAngleDiff)*0.00277777;
        m_RRSteerMotor.getClosedLoopController().setReference(m_RRSteerMotor_Ref,ControlType.kPosition, ClosedLoopSlot.kSlot0);
 
        
        // ---------- Set Drive Motor Speeds
        
        // go ahead and set motor closed loop target speeds
        m_LFDriveMotor_Ref = m_states[0].speedMetersPerSecond*LFDriveDir*MPS_TO_WHEELRPM;
        m_LFDriveMotor.getClosedLoopController().setReference(m_LFDriveMotor_Ref,ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        m_RFDriveMotor_Ref = m_states[1].speedMetersPerSecond*RFDriveDir*MPS_TO_WHEELRPM;
        m_RFDriveMotor.getClosedLoopController().setReference(m_RFDriveMotor_Ref,ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        m_LRDriveMotor_Ref = m_states[2].speedMetersPerSecond*LRDriveDir*MPS_TO_WHEELRPM;
        m_LRDriveMotor.getClosedLoopController().setReference(m_LRDriveMotor_Ref,ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        m_RRDriveMotor_Ref = m_states[3].speedMetersPerSecond*RRDriveDir*MPS_TO_WHEELRPM;
        m_RRDriveMotor.getClosedLoopController().setReference(m_RRDriveMotor_Ref,ControlType.kVelocity, ClosedLoopSlot.kSlot0);

        

    }


    // returns positions of all swerve modules
    public SwerveModulePosition[] GetSwerveDistances() {
 
        // create array of module positions to return
        SwerveModulePosition[] states = new SwerveModulePosition[4];
            
        // populate distance(m) and angle for LF swerve (note: rps_to_mps same as r_to_m)
        states[0] = new SwerveModulePosition();
        states[0].distanceMeters = m_LFDriveMotor.getEncoder().getPosition() * WHEELRPS_TO_MPS;
        states[0].angle = Rotation2d.fromDegrees(m_LFSteerMotor.getEncoder().getPosition()*360.0);

        // populate distance(m) and angle for RF swerve
        states[1] = new SwerveModulePosition();
        states[1].distanceMeters = m_RFDriveMotor.getEncoder().getPosition() * WHEELRPS_TO_MPS;
        states[1].angle = Rotation2d.fromDegrees(m_RFSteerMotor.getEncoder().getPosition()*360.0);

        // populate distance(m) and angle for LR swerve
        states[2] = new SwerveModulePosition();
        states[2].distanceMeters = m_LRDriveMotor.getEncoder().getPosition() * WHEELRPS_TO_MPS;
        states[2].angle = Rotation2d.fromDegrees(m_LRSteerMotor.getEncoder().getPosition()*360.0);

        // populate distance(m) and angle for RR swerve
        states[3] = new SwerveModulePosition();
        states[3].distanceMeters = m_RRDriveMotor.getEncoder().getPosition() * WHEELRPS_TO_MPS;
        states[3].angle = Rotation2d.fromDegrees(m_RRSteerMotor.getEncoder().getPosition()*360.0);
        
        return states;
    }

    // returns current states of all swerve modules
    public SwerveModuleState[] GetSwerveStates() {
        
        // create array of module positions to return
        SwerveModuleState[] states = new SwerveModuleState[4];

        // populate speed(m/s) and angle for LF swerve (note: rpm_to_mps same as r_to_m)
        
        states[0] = new SwerveModuleState();
        states[0].speedMetersPerSecond = m_LFDriveMotor.getEncoder().getVelocity() * WHEELRPM_TO_MPS;
        states[0].angle = Rotation2d.fromDegrees(m_LFSteerMotor.getEncoder().getPosition()*360.0);

        // populate speed(m/s) and angle for RF swerve
        states[1] = new SwerveModuleState();
        states[1].speedMetersPerSecond = m_RFDriveMotor.getEncoder().getVelocity() * WHEELRPM_TO_MPS;
        states[1].angle = Rotation2d.fromDegrees(m_RFSteerMotor.getEncoder().getPosition()*360.0);

        // populate speed(m/s) and angle for LR swerve
        states[2] = new SwerveModuleState();
        states[2].speedMetersPerSecond = m_LRDriveMotor.getEncoder().getVelocity() * WHEELRPM_TO_MPS;
        states[2].angle = Rotation2d.fromDegrees(m_LRSteerMotor.getEncoder().getPosition()*360.0);

        // populate speed(m/s) and angle for RR swerve
        states[3] = new SwerveModuleState();
        states[3].speedMetersPerSecond = m_RRDriveMotor.getEncoder().getVelocity() * WHEELRPM_TO_MPS;
        states[3].angle = Rotation2d.fromDegrees(m_RRSteerMotor.getEncoder().getPosition()*360.0);

        return states;
    }

    /* Returns commanded swerve target states - intended for use by drive simulation */
    public SwerveModuleState[] getTargetModuleStates() {
        return m_states;
    }

    /** Returns kinematics of drive system */
    public SwerveDriveKinematics getKinematics() {
        return driveKinematics;
    }


    // -------------------- Subsystem Shuffleboard Methods --------------------

    
    private GenericEntry m_LFCanCoderPos;
    private GenericEntry m_RFCanCoderPos;
    private GenericEntry m_LRCanCoderPos;
    private GenericEntry m_RRCanCoderPos;
    private GenericEntry m_LFSteerMotorPos;
    private GenericEntry m_RFSteerMotorPos;
    private GenericEntry m_LRSteerMotorPos;
    private GenericEntry m_RRSteerMotorPos;
    private GenericEntry m_LFSteerMotorTarget;
    private GenericEntry m_RFSteerMotorTarget;
    private GenericEntry m_LRSteerMotorTarget;
    private GenericEntry m_RRSteerMotorTarget;
    private GenericEntry m_LFDriveMotorPos;
    private GenericEntry m_RFDriveMotorPos;
    private GenericEntry m_LRDriveMotorPos;
    private GenericEntry m_RRDriveMotorPos;
    private GenericEntry m_LFDriveMotorTargetSpeed;
    private GenericEntry m_RFDriveMotorTargetSpeed;
    private GenericEntry m_LRDriveMotorTargetSpeed;
    private GenericEntry m_RRDriveMotorTargetSpeed;
    private GenericEntry m_LFDriveMotorSpeed;
    private GenericEntry m_RFDriveMotorSpeed;
    private GenericEntry m_LRDriveMotorSpeed;
    private GenericEntry m_RRDriveMotorSpeed;
    
    
    /** Initialize subsystem shuffleboard page and controls */
    private void initializeShuffleboard() {
        // Create odometry page in shuffleboard
        ShuffleboardTab Tab = Shuffleboard.getTab("SwerveDrive");

        // create button/command to reset swerve steering
        Tab.add("Reset Swerve", new InstantCommand(()->{ResetSteerEncoders();}))
            .withPosition(4,0)
            .withSize(2, 1);

        // create controls to left-front swerve data
        ShuffleboardLayout l1 = Tab.getLayout("Left-Front", BuiltInLayouts.kList);
        l1.withPosition(0, 0);
        l1.withSize(1, 5);
        m_LFCanCoderPos = l1.add("Sensor(deg)", 0.0).getEntry();
        m_LFSteerMotorPos = l1.add("Steer Pos(deg)", 0.0).getEntry();
        m_LFSteerMotorTarget = l1.add("Steer Target(deg)", 0.0).getEntry();
        m_LFDriveMotorPos = l1.add("Drive Pos(m)", 0.0).getEntry();
        m_LFDriveMotorSpeed = l1.add("Drive Speed(mps)", 0.0).getEntry();
        m_LFDriveMotorTargetSpeed = l1.add("Drive Target(mps)", 0.0).getEntry();

        // create controls to right-front swerve data
        ShuffleboardLayout l2 = Tab.getLayout("Right-Front", BuiltInLayouts.kList);
        l2.withPosition(1, 0);
        l2.withSize(1, 5);
        m_RFCanCoderPos = l2.add("Sensor(deg)", 0.0).getEntry();
        m_RFSteerMotorPos = l2.add("Steer Pos(deg)", 0.0).getEntry();
        m_RFSteerMotorTarget = l2.add("Steer Target(deg)", 0.0).getEntry();
        m_RFDriveMotorPos = l2.add("Drive Pos(m)", 0.0).getEntry();
        m_RFDriveMotorSpeed = l2.add("Drive Speed(mps)", 0.0).getEntry();
        m_RFDriveMotorTargetSpeed = l2.add("Drive Target(mps)", 0.0).getEntry();

        // create controls to left-rear swerve data
        ShuffleboardLayout l3 = Tab.getLayout("Left-Rear", BuiltInLayouts.kList);
        l3.withPosition(2, 0);
        l3.withSize(1, 5);
        m_LRCanCoderPos = l3.add("Sensor(deg)", 0.0).getEntry();
        m_LRSteerMotorPos = l3.add("Steer Pos(deg)", 0.0).getEntry();
        m_LRSteerMotorTarget = l3.add("Steer Target(deg)", 0.0).getEntry();
        m_LRDriveMotorPos = l3.add("Drive Pos(m)", 0.0).getEntry();
        m_LRDriveMotorSpeed = l3.add("Drive Speed(mps)", 0.0).getEntry();
        m_LRDriveMotorTargetSpeed = l3.add("Drive Target(mps)", 0.0).getEntry();

        // create controls to right-rear swerve data
        ShuffleboardLayout l4 = Tab.getLayout("Right-Rear", BuiltInLayouts.kList);
        l4.withPosition(3, 0);
        l4.withSize(1, 5);
        m_RRCanCoderPos = l4.add("Sensor(deg)", 0.0).getEntry();
        m_RRSteerMotorPos = l4.add("Steer Pos(deg)", 0.0).getEntry();
        m_RRSteerMotorTarget = l4.add("Steer Target(deg)", 0.0).getEntry();
        m_RRDriveMotorPos = l4.add("Drive Pos(m)", 0.0).getEntry();
        m_RRDriveMotorSpeed = l4.add("Drive Speed(mps)", 0.0).getEntry();
        m_RRDriveMotorTargetSpeed = l4.add("Drive Target(mps)", 0.0).getEntry();
    }


    /** Update subsystem shuffle board page with current Gyro values */
    private void updateShuffleboard() {
    
        // update steer position values (deg)
        m_LFCanCoderPos.setDouble(-m_LFAngleSensor.get()*360.0);
        m_RFCanCoderPos.setDouble(-m_RFAngleSensor.get()*360.0);
        m_LRCanCoderPos.setDouble(-m_LRAngleSensor.get()*360.0);
        m_RRCanCoderPos.setDouble(-m_RRAngleSensor.get()*360.0);

        // update steer motor position values (deg)
        m_LFSteerMotorPos.setDouble(m_LFSteerMotor.getEncoder().getPosition()*360.0);
        m_RFSteerMotorPos.setDouble(m_RFSteerMotor.getEncoder().getPosition()*360.0);
        m_LRSteerMotorPos.setDouble(m_LRSteerMotor.getEncoder().getPosition()*360.0);
        m_RRSteerMotorPos.setDouble(m_RRSteerMotor.getEncoder().getPosition()*360.0);

        // update steer motor target values (deg)
        m_LFSteerMotorTarget.setDouble(m_LFSteerMotor_Ref*360.0);
        m_RFSteerMotorTarget.setDouble(m_RFSteerMotor_Ref*360.0);
        m_LRSteerMotorTarget.setDouble(m_LRSteerMotor_Ref*360.0);
        m_RRSteerMotorTarget.setDouble(m_RRSteerMotor_Ref*360.0);

        // update drive motor speeds (m/s)
        m_LFDriveMotorSpeed.setDouble(m_LFDriveMotor.getEncoder().getVelocity()*WHEELRPM_TO_MPS);
        m_RFDriveMotorSpeed.setDouble(m_RFDriveMotor.getEncoder().getVelocity()*WHEELRPM_TO_MPS);
        m_LRDriveMotorSpeed.setDouble(m_LRDriveMotor.getEncoder().getVelocity()*WHEELRPM_TO_MPS);
        m_RRDriveMotorSpeed.setDouble(m_RRDriveMotor.getEncoder().getVelocity()*WHEELRPM_TO_MPS);

        // update drive motor targets (m/s)
        m_LFDriveMotorTargetSpeed.setDouble(m_LFDriveMotor_Ref*WHEELRPM_TO_MPS);
        m_RFDriveMotorTargetSpeed.setDouble(m_RFDriveMotor_Ref*WHEELRPM_TO_MPS);
        m_LRDriveMotorTargetSpeed.setDouble(m_LRDriveMotor_Ref*WHEELRPM_TO_MPS);
        m_RRDriveMotorTargetSpeed.setDouble(m_RRDriveMotor_Ref*WHEELRPM_TO_MPS);

        // update drive motor position values (m)
        m_LFDriveMotorPos.setDouble(m_LFDriveMotor.getEncoder().getPosition()*WHEELRPS_TO_MPS);
        m_RFDriveMotorPos.setDouble(m_RFDriveMotor.getEncoder().getPosition()*WHEELRPS_TO_MPS);
        m_LRDriveMotorPos.setDouble(m_LRDriveMotor.getEncoder().getPosition()*WHEELRPS_TO_MPS);
        m_RRDriveMotorPos.setDouble(m_RRDriveMotor.getEncoder().getPosition()*WHEELRPS_TO_MPS);
    }



}

