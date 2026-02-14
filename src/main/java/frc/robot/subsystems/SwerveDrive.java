package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.commands.ManualDrive;
import frc.robot.utils.Utils;

/** Subsystem */
public class SwerveDrive extends SubsystemBase {


    // Setup Drive Kinematics Object Constants
    private final double TRACK_WIDTH = 0.5969; // Width between the left and right wheels - in m.
    private final double TRACK_LENGTH = 0.4445; // Length between the front and back wheel - in m.
    
    // steering gear ratio for SPS MK4 L1 Swerve
    private final double STEER_RATIO = 12.8;

    // drive gear ratio for SPS MK4 L1 Swerve
    private final double DRIVE_RATIO = 8.14;

    // drive wheel diameter in m
    private double WHEEL_DIA = 0.1;              

    // conversion factors - used to convert motor MPS to RPS and vice versa
    private final double WHEELRPS_TO_MPS = Math.PI * WHEEL_DIA;
    private final double MPS_TO_WHEELRPS = 1.0 / WHEELRPS_TO_MPS;

    // constants for FalconFx motors
    private final double MAXRPM = 6380.0*0.975;
    private final double MAXRPS = MAXRPM / 60.0;

    // maximum expected drive speed or robot (m/s)
    public final double MAX_SPEED = MAXRPS * WHEELRPS_TO_MPS / DRIVE_RATIO;

     // The drivetrain's kinematics model
    private SwerveDriveKinematics driveKinematics;
    private final Translation2d m_CenterOfRotation = new Translation2d(0.0,0.0);;
    private final Rotation2d m_ParkAngleLF = new Rotation2d(45.0);
    private final Rotation2d m_ParkAngleRF = new Rotation2d(-45.0);

    // create CANCoder sensor objects
    private CANcoder m_LFCanCoder;
    private CANcoder m_RFCanCoder;
    private CANcoder m_LRCanCoder;
    private CANcoder m_RRCanCoder;

    // create steer motor objects
    // create position controllers for steer motors
    private TalonFX m_LFSteerMotor;
    private TalonFX m_RFSteerMotor;
    private TalonFX m_LRSteerMotor;
    private TalonFX m_RRSteerMotor;
    private PositionVoltage m_LFSteerControl;
    private PositionVoltage m_RFSteerControl;
    private PositionVoltage m_LRSteerControl;
    private PositionVoltage m_RRSteerControl;

    // create drive motor objects
    // create speed controllers for drive motors
    private TalonFX m_LFDriveMotor;
    private TalonFX m_RFDriveMotor;
    private TalonFX m_LRDriveMotor;
    private TalonFX m_RRDriveMotor;
    private VelocityVoltage m_LFDriveControl;
    private VelocityVoltage m_RFDriveControl;
    private VelocityVoltage m_LRDriveControl;
    private VelocityVoltage m_RRDriveControl;

    private final Pigeon gyro;

    // Swerve module states - contains target speed(m/s) and angle for each swerve module
    private SwerveModuleState[] m_states;

    
    public void initDefaultCommand() {
    }

    /** Place code here to initialize subsystem */
    public SwerveDrive(Pigeon gyro) {
       this.gyro = gyro;

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


        // ---------- CANCoders Setup ----------

        // create CANCoder objects - cancoders used for absolute steering feedback
        m_LFCanCoder = new CANcoder(RobotMap.CANID.LF_CANCODER);
        m_RFCanCoder = new CANcoder(RobotMap.CANID.RF_CANCODER);
        m_LRCanCoder = new CANcoder(RobotMap.CANID.LR_CANCODER);
        m_RRCanCoder = new CANcoder(RobotMap.CANID.RR_CANCODER);

        // configure cancoders
        // configure for counter-clockwise positive to match FRC coorinate system
        // set to provide values between -0.5 and 0.5
        // Note: must configure separately as each cancoder will have a unique offset to be set
        CANcoderConfiguration LFEncoderConfig = new CANcoderConfiguration();
        LFEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        LFEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        CANcoderConfiguration RFEncoderConfig = new CANcoderConfiguration();
        RFEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        RFEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        CANcoderConfiguration LREncoderConfig = new CANcoderConfiguration();
        LREncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        LREncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        CANcoderConfiguration RREncoderConfig = new CANcoderConfiguration();
        RREncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        RREncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        
        // encoder offsets - ADJUST values to align wheels
        // units - in rotations
        // note:  if +ve value when wheels are aligned straight, then decrease offset by that value
        // encoders should then read zero when aligned straight.
        // when aligning, wheel ring gears face inside robot
        // multiply by factor to convert deg back to rotations
        LFEncoderConfig.MagnetSensor.MagnetOffset = -183.73 * 0.00277777;
        RFEncoderConfig.MagnetSensor.MagnetOffset = 112.5 * 0.00277777;
        LREncoderConfig.MagnetSensor.MagnetOffset = -14.8 * 0.00277777;
        RREncoderConfig.MagnetSensor.MagnetOffset = 10.1 * 0.00277777;

        // apply configuration to cancoders
        m_LFCanCoder.getConfigurator().apply(LFEncoderConfig);
        m_RFCanCoder.getConfigurator().apply(RFEncoderConfig);
        m_LRCanCoder.getConfigurator().apply(LREncoderConfig);
        m_RRCanCoder.getConfigurator().apply(RREncoderConfig);
        

        // ---------- Steer Motor Setup ----------

        // create steer motors
        m_LFSteerMotor = new TalonFX(RobotMap.CANID.LF_STEER_MOTOR);
        m_RFSteerMotor = new TalonFX(RobotMap.CANID.RF_STEER_MOTOR);
        m_LRSteerMotor = new TalonFX(RobotMap.CANID.LR_STEER_MOTOR);
        m_RRSteerMotor = new TalonFX(RobotMap.CANID.RR_STEER_MOTOR);

        // turn on safety oversight of steer motors
        m_LFSteerMotor.setSafetyEnabled(false);
        m_RFSteerMotor.setSafetyEnabled(true);
        m_LRSteerMotor.setSafetyEnabled(true);
        m_RRSteerMotor.setSafetyEnabled(true);

        // configure steer motors
        // set for counter-clockwise +ve rotation - to match FRC coordinate system
        // set neutral mode to coast.  Can change to brake later for competition use
        // set deadband so steer motor does not chatter when at destination position
        // used CTRE webpage to estimate gains from Phoenix 5 gains used in 2023
        // set sensor to mechanism gear ratio
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        steerConfig.MotorOutput.DutyCycleNeutralDeadband = 0.0001;
        steerConfig.Slot0.kP = 60.0;  //volts/rot  =70.0 Feb 25/2025
        steerConfig.Slot0.kI = 10.0;  //volts/rot-s
        steerConfig.Slot0.kD = 0.0;   //volts/ros/s
        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        steerConfig.Feedback.SensorToMechanismRatio = STEER_RATIO;
        //steerConfig.CurrentLimits.SupplyCurrentLimitEnable =  // note default is true
        //steerConfig.CurrentLimits.SupplyCurrentLimit =        // note default is 80A
        //steerConfig.CurrentLimits.SupplyCurrentLowerTime =    // note default is 1.0s
        //steerConfig.CurrentLimits.SupplyCurrentLowerLimit =   // note default is 40A

        // apply configuration to motors
        m_LFSteerMotor.getConfigurator().apply(steerConfig);
        m_RFSteerMotor.getConfigurator().apply(steerConfig);
        m_LRSteerMotor.getConfigurator().apply(steerConfig);
        m_RRSteerMotor.getConfigurator().apply(steerConfig);

        // create steer motor feedback controllers
        m_LFSteerControl = new PositionVoltage(0.0);
        m_RFSteerControl = new PositionVoltage(0.0);
        m_LRSteerControl = new PositionVoltage(0.0);
        m_RRSteerControl = new PositionVoltage(0.0);
        
        // set up steer motor controllers
        // use SLot0 for PID values
        // set initial targert position to 0.0
        m_LFSteerControl.Slot = 0;
        m_RFSteerControl.Slot = 0;
        m_LRSteerControl.Slot = 0;
        m_RRSteerControl.Slot = 0;
        m_LFSteerControl.Position = 0.0;
        m_RFSteerControl.Position = 0.0;
        m_LRSteerControl.Position = 0.0;
        m_RRSteerControl.Position = 0.0;

        
        // ---------- Drive Motor Setup ----------


        // create drive motors
        m_LFDriveMotor = new TalonFX(RobotMap.CANID.LF_DRIVE_MOTOR);
        m_RFDriveMotor = new TalonFX(RobotMap.CANID.RF_DRIVE_MOTOR);
        m_LRDriveMotor = new TalonFX(RobotMap.CANID.LR_DRIVE_MOTOR);
        m_RRDriveMotor = new TalonFX(RobotMap.CANID.RR_DRIVE_MOTOR);

        // turn on safety of all drive motors
        m_LFDriveMotor.setSafetyEnabled(true);
        m_RFDriveMotor.setSafetyEnabled(true);
        m_LRDriveMotor.setSafetyEnabled(true);
        m_RRDriveMotor.setSafetyEnabled(true);

        // configure drive motors
        // set neutral mode to coast.  Can change to brake later for competition use
        // set deadband so drive motor does not chatter when at zero speed
        // used CTRE webpage to estimate gains from Phoenix 5 gains used in 2023
        // set sensor to mechanism gear ratio
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.MotorOutput.DutyCycleNeutralDeadband = 0.0001;
        driveConfig.Slot0.kP = 2.0;    // volts/rps
        driveConfig.Slot0.kI = 0.0;    // volts/rps-s
        driveConfig.Slot0.kD = 0.0;    // volts/rps/s
        driveConfig.Slot0.kS = 0.08;   // volts
        driveConfig.Slot0.kV = 0.916;  // volts/rps
        driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveConfig.Feedback.SensorToMechanismRatio = DRIVE_RATIO;
        //steerConfig.CurrentLimits.SupplyCurrentLimitEnable =  // note default is true
        //steerConfig.CurrentLimits.SupplyCurrentLimit =        // note default is 80A
        //steerConfig.CurrentLimits.SupplyCurrentLowerTime =    // note default is 1.0s
        //steerConfig.CurrentLimits.SupplyCurrentLowerLimit =   // note default is 40A

        // apply configuration to motors - set motor +ve direction depending on motor
        driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_LFDriveMotor.getConfigurator().apply(driveConfig);
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_RFDriveMotor.getConfigurator().apply(driveConfig);
        driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_LRDriveMotor.getConfigurator().apply(driveConfig);
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_RRDriveMotor.getConfigurator().apply(driveConfig);

        // create drive motor feedback controllers
        m_LFDriveControl = new VelocityVoltage(0.0);
        m_RFDriveControl = new VelocityVoltage(0.0);
        m_LRDriveControl = new VelocityVoltage(0.0);
        m_RRDriveControl = new VelocityVoltage(0.0);

        // set up drive motor controllers
        // use SLot0 for PID values
        // set initial targert speed to 0.0
        m_LFDriveControl.Slot = 0;
        m_RFDriveControl.Slot = 0;
        m_LRDriveControl.Slot = 0;
        m_RRDriveControl.Slot = 0;
        m_LFDriveControl.Velocity = 0.0;
        m_RFDriveControl.Velocity = 0.0;
        m_LRDriveControl.Velocity = 0.0;
        m_RRDriveControl.Velocity = 0.0;
        
        
        m_LFDriveMotor.setControl(m_LFDriveControl.withVelocity(0.0));
        m_RFDriveMotor.setControl(m_RFDriveControl.withVelocity(0.0));
        m_LRDriveMotor.setControl(m_LRDriveControl.withVelocity(0.0));
        m_RRDriveMotor.setControl(m_RRDriveControl.withVelocity(0.0));



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
        m_LFSteerMotor.setPosition(m_LFCanCoder.getAbsolutePosition().getValue());
        m_RFSteerMotor.setPosition(m_RFCanCoder.getAbsolutePosition().getValue());
        m_LRSteerMotor.setPosition(m_LRCanCoder.getAbsolutePosition().getValue());
        m_RRSteerMotor.setPosition(m_RRCanCoder.getAbsolutePosition().getValue());
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
        ChassisSpeeds newSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(speed, Rotation2d.fromDegrees(gyro.getYawAngle()));
    
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
        double LFCurrentAngleDeg = m_LFSteerMotor.getPosition().getValueAsDouble()*360.0;

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
        m_LFSteerMotor.setControl(m_LFSteerControl.withPosition((LFCurrentAngleDeg + LFAngleDiff)*0.00277777));
        
      
        // ---------- Angle Determination for RF Swerve

        
        double RFCurrentAngleDeg = m_RFSteerMotor.getPosition().getValueAsDouble()*360.0;
        double RFTargetAngleDeg = m_states[1].angle.getDegrees();
        double RFAngleDiff = Utils.AngleDifference(RFCurrentAngleDeg, RFTargetAngleDeg);
        if (RFAngleDiff<-90.0)
            { RFDriveDir *= -1.0; RFAngleDiff+=180.0; }
        else if (RFAngleDiff>90)
            { RFDriveDir *= -1.0; RFAngleDiff-=180.0; }
        m_RFSteerMotor.setControl(m_RFSteerControl.withPosition((RFCurrentAngleDeg + RFAngleDiff)*0.00277777));


        // ---------- Angle Determination for LR Swerve

        
        double LRCurrentAngleDeg = m_LRSteerMotor.getPosition().getValueAsDouble()*360.0;
        double LRTargetAngleDeg = m_states[2].angle.getDegrees();
        double LRAngleDiff = Utils.AngleDifference(LRCurrentAngleDeg, LRTargetAngleDeg);
        if (LRAngleDiff<-90.0)
            { LRDriveDir *= -1.0; LRAngleDiff+=180.0; }
        else if (LRAngleDiff>90)
            { LRDriveDir *= -1.0; LRAngleDiff-=180.0; }
        m_LRSteerMotor.setControl(m_LRSteerControl.withPosition((LRCurrentAngleDeg + LRAngleDiff)*0.00277777));

        
        // ---------- Angle Determination for RR Swerve

        double RRCurrentAngleDeg = m_RRSteerMotor.getPosition().getValueAsDouble()*360.0;
        double RRTargetAngleDeg = m_states[3].angle.getDegrees();
        double RRAngleDiff = Utils.AngleDifference(RRCurrentAngleDeg, RRTargetAngleDeg);
        if (RRAngleDiff<-90.0)
            { RRDriveDir *= -1.0; RRAngleDiff+=180.0; }
        else if (RRAngleDiff>90)
            { RRDriveDir *= -1.0; RRAngleDiff-=180.0; }
        m_RRSteerMotor.setControl(m_RRSteerControl.withPosition((RRCurrentAngleDeg + RRAngleDiff)*0.00277777));
         

        // ---------- Set Drive Motor Speeds

        
        // go ahead and set motor closed loop target speeds
        m_LFDriveMotor.setControl(m_LFDriveControl.withVelocity(m_states[0].speedMetersPerSecond*LFDriveDir*MPS_TO_WHEELRPS));
        m_RFDriveMotor.setControl(m_RFDriveControl.withVelocity(m_states[1].speedMetersPerSecond*RFDriveDir*MPS_TO_WHEELRPS));
        m_LRDriveMotor.setControl(m_LRDriveControl.withVelocity(m_states[2].speedMetersPerSecond*LRDriveDir*MPS_TO_WHEELRPS));
        m_RRDriveMotor.setControl(m_RRDriveControl.withVelocity(m_states[3].speedMetersPerSecond*RRDriveDir*MPS_TO_WHEELRPS));
    }



    // returns positions of all swerve modules
    public SwerveModulePosition[] GetSwerveDistances() {
 
        // create array of module positions to return
        SwerveModulePosition[] states = new SwerveModulePosition[4];
            
        // populate distance(m) and angle for LF swerve (note: rps_to_mps same as r_to_m)
        states[0] = new SwerveModulePosition();
        states[0].distanceMeters = m_LFDriveMotor.getPosition().getValueAsDouble() * WHEELRPS_TO_MPS;
        states[0].angle = Rotation2d.fromDegrees(m_LFSteerMotor.getPosition().getValueAsDouble()*360.0);

        // populate distance(m) and angle for RF swerve
        states[1] = new SwerveModulePosition();
        states[1].distanceMeters = m_RFDriveMotor.getPosition().getValueAsDouble() * WHEELRPS_TO_MPS;
        states[1].angle = Rotation2d.fromDegrees(m_RFSteerMotor.getPosition().getValueAsDouble()*360.0);

        // populate distance(m) and angle for LR swerve
        states[2] = new SwerveModulePosition();
        states[2].distanceMeters = m_LRDriveMotor.getPosition().getValueAsDouble() * WHEELRPS_TO_MPS;
        states[2].angle = Rotation2d.fromDegrees(m_LRSteerMotor.getPosition().getValueAsDouble()*360.0);

        // populate distance(m) and angle for RR swerve
        states[3] = new SwerveModulePosition();
        states[3].distanceMeters = m_RRDriveMotor.getPosition().getValueAsDouble() * WHEELRPS_TO_MPS;
        states[3].angle = Rotation2d.fromDegrees(m_RRSteerMotor.getPosition().getValueAsDouble()*360.0);

        return states;
    }


    // returns current states of all swerve modules
    public SwerveModuleState[] GetSwerveStates() {
        
        // create array of module positions to return
        SwerveModuleState[] states = new SwerveModuleState[4];

        // populate speed(m/s) and angle for LF swerve (note: rps_to_mps same as r_to_m)
        states[0] = new SwerveModuleState();
        states[0].speedMetersPerSecond = m_LFDriveMotor.getVelocity().getValueAsDouble() * WHEELRPS_TO_MPS;
        states[0].angle = Rotation2d.fromDegrees(m_LFSteerMotor.getPosition().getValueAsDouble()*360.0);

        // populate speed(m/s) and angle for RF swerve
        states[1] = new SwerveModuleState();
        states[1].speedMetersPerSecond = m_RFDriveMotor.getVelocity().getValueAsDouble() * WHEELRPS_TO_MPS;
        states[1].angle = Rotation2d.fromDegrees(m_RFSteerMotor.getPosition().getValueAsDouble()*360.0);

        // populate speed(m/s) and angle for LR swerve
        states[2] = new SwerveModuleState();
        states[2].speedMetersPerSecond = m_LRDriveMotor.getVelocity().getValueAsDouble() * WHEELRPS_TO_MPS;
        states[2].angle = Rotation2d.fromDegrees(m_LRSteerMotor.getPosition().getValueAsDouble()*360.0);

        // populate speed(m/s) and angle for RR swerve
        states[3] = new SwerveModuleState();
        states[3].speedMetersPerSecond = m_RRDriveMotor.getVelocity().getValueAsDouble() * WHEELRPS_TO_MPS;
        states[3].angle = Rotation2d.fromDegrees(m_RRSteerMotor.getPosition().getValueAsDouble()*360.0);

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
        m_LFCanCoderPos = l1.add("CanCoder(deg)", 0.0).getEntry();
        m_LFSteerMotorPos = l1.add("Steer Pos(deg)", 0.0).getEntry();
        m_LFSteerMotorTarget = l1.add("Steer Target(deg)", 0.0).getEntry();
        m_LFDriveMotorPos = l1.add("Drive Pos(m)", 0.0).getEntry();
        m_LFDriveMotorSpeed = l1.add("Drive Speed(mps)", 0.0).getEntry();
        m_LFDriveMotorTargetSpeed = l1.add("Drive Target(mps)", 0.0).getEntry();

        // create controls to right-front swerve data
        ShuffleboardLayout l2 = Tab.getLayout("Right-Front", BuiltInLayouts.kList);
        l2.withPosition(1, 0);
        l2.withSize(1, 5);
        m_RFCanCoderPos = l2.add("CanCoder(deg)", 0.0).getEntry();
        m_RFSteerMotorPos = l2.add("Steer Pos(deg)", 0.0).getEntry();
        m_RFSteerMotorTarget = l2.add("Steer Target(deg)", 0.0).getEntry();
        m_RFDriveMotorPos = l2.add("Drive Pos(m)", 0.0).getEntry();
        m_RFDriveMotorSpeed = l2.add("Drive Speed(mps)", 0.0).getEntry();
        m_RFDriveMotorTargetSpeed = l2.add("Drive Target(mps)", 0.0).getEntry();

        // create controls to left-rear swerve data
        ShuffleboardLayout l3 = Tab.getLayout("Left-Rear", BuiltInLayouts.kList);
        l3.withPosition(2, 0);
        l3.withSize(1, 5);
        m_LRCanCoderPos = l3.add("CanCoder(deg)", 0.0).getEntry();
        m_LRSteerMotorPos = l3.add("Steer Pos(deg)", 0.0).getEntry();
        m_LRSteerMotorTarget = l3.add("Steer Target(deg)", 0.0).getEntry();
        m_LRDriveMotorPos = l3.add("Drive Pos(m)", 0.0).getEntry();
        m_LRDriveMotorSpeed = l3.add("Drive Speed(mps)", 0.0).getEntry();
        m_LRDriveMotorTargetSpeed = l3.add("Drive Target(mps)", 0.0).getEntry();

        // create controls to right-rear swerve data
        ShuffleboardLayout l4 = Tab.getLayout("Right-Rear", BuiltInLayouts.kList);
        l4.withPosition(3, 0);
        l4.withSize(1, 5);
        m_RRCanCoderPos = l4.add("CanCoder(deg)", 0.0).getEntry();
        m_RRSteerMotorPos = l4.add("Steer Pos(deg)", 0.0).getEntry();
        m_RRSteerMotorTarget = l4.add("Steer Target(deg)", 0.0).getEntry();
        m_RRDriveMotorPos = l4.add("Drive Pos(m)", 0.0).getEntry();
        m_RRDriveMotorSpeed = l4.add("Drive Speed(mps)", 0.0).getEntry();
        m_RRDriveMotorTargetSpeed = l4.add("Drive Target(mps)", 0.0).getEntry();

    }


    /** Update subsystem shuffle board page with current Gyro values */
    private void updateShuffleboard() {
    
        // update CANCoder position values (deg)
        m_LFCanCoderPos.setDouble(m_LFCanCoder.getAbsolutePosition().getValueAsDouble()*360.0);
        m_RFCanCoderPos.setDouble(m_RFCanCoder.getAbsolutePosition().getValueAsDouble()*360.0);
        m_LRCanCoderPos.setDouble(m_LRCanCoder.getAbsolutePosition().getValueAsDouble()*360.0);
        m_RRCanCoderPos.setDouble(m_RRCanCoder.getAbsolutePosition().getValueAsDouble()*360.0);
    
        // update steer motor position values (deg)
        m_LFSteerMotorPos.setDouble(m_LFSteerMotor.getPosition().getValueAsDouble()*360.0);
        m_RFSteerMotorPos.setDouble(m_RFSteerMotor.getPosition().getValueAsDouble()*360.0);
        m_LRSteerMotorPos.setDouble(m_LRSteerMotor.getPosition().getValueAsDouble()*360.0);
        m_RRSteerMotorPos.setDouble(m_RRSteerMotor.getPosition().getValueAsDouble()*360.0);

        // update steer motor target values (deg)
        m_LFSteerMotorTarget.setDouble(m_LFSteerMotor.getClosedLoopReference().getValueAsDouble()*360.0);
        m_RFSteerMotorTarget.setDouble(m_RFSteerMotor.getClosedLoopReference().getValueAsDouble()*360.0);
        m_LRSteerMotorTarget.setDouble(m_LRSteerMotor.getClosedLoopReference().getValueAsDouble()*360.0);
        m_RRSteerMotorTarget.setDouble(m_RRSteerMotor.getClosedLoopReference().getValueAsDouble()*360.0);

        // update drive motor speeds (m/s)
        m_LFDriveMotorSpeed.setDouble(m_LFDriveMotor.getVelocity().getValueAsDouble()*WHEELRPS_TO_MPS);
        m_RFDriveMotorSpeed.setDouble(m_RFDriveMotor.getVelocity().getValueAsDouble()*WHEELRPS_TO_MPS);
        m_LRDriveMotorSpeed.setDouble(m_LRDriveMotor.getVelocity().getValueAsDouble()*WHEELRPS_TO_MPS);
        m_RRDriveMotorSpeed.setDouble(m_RRDriveMotor.getVelocity().getValueAsDouble()*WHEELRPS_TO_MPS);

        // update drive motor targets (m/s)
        m_LFDriveMotorTargetSpeed.setDouble(m_LFDriveMotor.getClosedLoopReference().getValueAsDouble()*WHEELRPS_TO_MPS);
        m_RFDriveMotorTargetSpeed.setDouble(m_RFDriveMotor.getClosedLoopReference().getValueAsDouble()*WHEELRPS_TO_MPS);
        m_LRDriveMotorTargetSpeed.setDouble(m_LRDriveMotor.getClosedLoopReference().getValueAsDouble()*WHEELRPS_TO_MPS);
        m_RRDriveMotorTargetSpeed.setDouble(m_RRDriveMotor.getClosedLoopReference().getValueAsDouble()*WHEELRPS_TO_MPS);

        // update drive motor position values (m)
        m_LFDriveMotorPos.setDouble(m_LFDriveMotor.getPosition().getValueAsDouble()*WHEELRPS_TO_MPS);
        m_RFDriveMotorPos.setDouble(m_RFDriveMotor.getPosition().getValueAsDouble()*WHEELRPS_TO_MPS);
        m_LRDriveMotorPos.setDouble(m_LRDriveMotor.getPosition().getValueAsDouble()*WHEELRPS_TO_MPS);
        m_RRDriveMotorPos.setDouble(m_RRDriveMotor.getPosition().getValueAsDouble()*WHEELRPS_TO_MPS);

        // Log swerve states for AdvantageScope
        if (m_states != null) {
            double[] desiredStates = new double[m_states.length * 2];
            for (int i = 0; i < m_states.length; i++) {
                desiredStates[i * 2] = m_states[i].angle.getRadians();
                desiredStates[i * 2 + 1] = m_states[i].speedMetersPerSecond;
            }
            SmartDashboard.putNumberArray("Swerve/DesiredStates", desiredStates);
        }

        SwerveModuleState[] measuredStates = GetSwerveStates();
        double[] measuredStatesArray = new double[measuredStates.length * 2];
        for (int i = 0; i < measuredStates.length; i++) {
            measuredStatesArray[i * 2] = measuredStates[i].angle.getRadians();
            measuredStatesArray[i * 2 + 1] = measuredStates[i].speedMetersPerSecond;
        }
        SmartDashboard.putNumberArray("Swerve/MeasuredStates", measuredStatesArray);
    }



}
