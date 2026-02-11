package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight.LimelightResults;

/** Subsystem */
public class Odometry extends SubsystemBase {
  
    // timer used for simuation purposes only
    Timer simTimer;
  
    // constant to convert degrees to radians
    public final static float DEGtoRAD = (float) (3.1415926 / 180.0);
    static double previousLeft, previousFront, previousRear;
    
    // swerve position estimator
    private SwerveDrivePoseEstimator m_Estimator;

    // timer for pose estimator time-stamping
    private Timer PoseTimeStamp;
   
    // latest apriltag detections
    private LimelightResults TagResults;

    // time since last apriltag
    private Timer timeSinceLastTag;

    Pose2d currentPoseBeforeAdjustment;


    public boolean TagEnable = true;

    public Odometry() {

        // create position estimator - set to (0,0,0)(x,y,ang)
        // initialize swerve drive odometry
        m_Estimator = new SwerveDrivePoseEstimator(
            RobotContainer.drivesystem.getKinematics(),
            new Rotation2d(0),
            RobotContainer.drivesystem.GetSwerveDistances(),
            new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
            VecBuilder.fill(1.0, 1.0, 0.1),
            VecBuilder.fill(0.02, 0.02, 0.03));

        // reset initial position
        InitializeToZero();

        // create odometry shuffleboard page
        initializeShuffleboard();

        // reset/start timer
        PoseTimeStamp = new Timer();
        PoseTimeStamp.reset();
        PoseTimeStamp.start();

        // reset deadwheel encoders
        // RobotContainer.encoder.ResetEncoder();
        previousLeft = 0.0;
        previousFront = 0.0;
        previousRear = 0.0;

        // initialize simulation timer
        simTimer = new Timer();
        simTimer.reset();
        simTimer.start();

        timeSinceLastTag = new Timer();
        timeSinceLastTag.reset();
        timeSinceLastTag.start();


    }

    /**
    * Method called periodically by the scheduler
    * Place any code here you wish to have run periodically
    */
    @Override
    public void periodic() {
    
        // get current pose before adjustments
        currentPoseBeforeAdjustment = getPose2d();
        
        // pose update using drive wheel data
        updateDriveWheelOdometry();
        
        // pose update using dead-wheel data
        //updateDeadWheelOdometry();

        // pose update using apriltag data
        if (TagEnable){
            // updateAprilTagOdometry(RobotContainer.camleft);
            // updateAprilTagOdometry(RobotContainer.camr);
        }
       


        updateShuffleboard();
    
        
    }


    // -------- get/update odometry methods

    /** return robot's current position vector Pose2d */
    public Pose2d getPose2d() {
        return m_Estimator.getEstimatedPosition();
    }
    
    // initialize robot odometry to zero
    public void InitializeToZero() {
        setPose(0.0, 0.0, 0.0, 0.0);
    }

    /** Use to set odometry to fixed position and angle */
    public void setPose(Pose2d position) {
        setPose(position, position.getRotation()); }
    public void setPose(Pose2d position, Rotation2d gyroangle) {
        //temp
        DWfieldX = position.getX();
        DWfieldY = position.getY();
        DWfieldAngle = position.getRotation().getRadians();
        
        // set gyro
        RobotContainer.gyro.setYawAngle(gyroangle.getDegrees());
    
        // set robot odometry
        m_Estimator.resetPosition(gyroangle,
            RobotContainer.drivesystem.GetSwerveDistances(),
            position);
    }
  
    public void setPose(double x, double y, double robotangle) {
        setPose(x, y, robotangle, robotangle);
    }
    public void setPose(double x, double y, double robotangle, double gyroangle) {
        setPose(new Pose2d(x,y, new Rotation2d(robotangle)), new Rotation2d(gyroangle)); }


    // ---------- Drive-wheel odometry methods ----------
    
    
    // helper function to updates drive wheel odometry - called by periodic()
    private void updateDriveWheelOdometry() {
        // get gyro angle (in degrees) and make rotation vector
        Rotation2d gyroangle = new Rotation2d(RobotContainer.gyro.getYawAngle() * DEGtoRAD);

        // get position of all sewrve modules from subsystem
        SwerveModulePosition[] positions = RobotContainer.drivesystem.GetSwerveDistances();

        // ensure we have the proper length array positions
        if (positions.length >= 4) {

            // update robots odometry
            m_Estimator.updateWithTime(PoseTimeStamp.get(), gyroangle, positions);

        }
    }
    
    
    // ---------- Dead-wheel update odometry methods ----------

    double DWfieldX = 0.0;
    double DWfieldY = 0.0;
    double DWfieldAngle = 0.0;

    // helper function to update dead wheel odometry - called by periodic()
    // private void updateDeadWheelOdometry() {
    
    //     // get all the deadwheel encoder distances (in m)
    //     //double leftPos = RobotContainer.encoder.getLeftEncoderDistance()*0.01;
    //     //double frontPos = RobotContainer.encoder.getFrontEncoderDistance()*0.01;
    //     //double rearPos = RobotContainer.encoder.getRearEncoderDistance()*0.01;

    //     // determine changes in distances.
    //     //double leftChangePos = leftPos - previousLeft;
    //     //double frontChangePos = frontPos - previousFront;
    //     //double rearChangePos = rearPos - previousRear;

    //     // keep encoder positions for next time
    //     //previousLeft = leftPos;
    //     //previousFront = frontPos;
    //     //previousRear = rearPos;

    //     // creating the value of sin theta (aka the angle of the hipotinuse)
    //     //double theta = Math.asin((frontChangePos - rearChangePos) / DeadWheel.FRONT_TO_BACK_DISTANCE);

    //     // equation that tells us how much the robot has moved sideways (=avg of front and back encoders)
    //     double LateralChange = (frontChangePos + rearChangePos) / 2.0;

    //     // equation that tells us how much the robot has moved forward 
    //     //double ForwardChange = (leftChangePos + DeadWheel.LATERAL_OFFSET * Math.sin(theta));


    //     // need to convert lateral and forward movement from robot-orientation to field-orientation
    //     // based on angle of gyro
    //     double IMUHeading = Math.toRadians(RobotContainer.gyro.getYawAngle());

    //     double fieldForwardChange = ForwardChange * Math.cos(IMUHeading) - LateralChange * Math.sin(IMUHeading);

    //     double fieldLateralChange = ForwardChange * Math.sin(IMUHeading) + LateralChange * Math.cos(IMUHeading);

        
        
        
        
    //     Pose2d newPose = new Pose2d(currentPoseBeforeAdjustment.getX()+fieldForwardChange,
    //                                 currentPoseBeforeAdjustment.getY()+fieldLateralChange,
    //                                 new Rotation2d(IMUHeading));
           
    //     // temp
    //     DWfieldX += fieldForwardChange;
    //     DWfieldY += fieldLateralChange;
    //     DWfieldAngle = IMUHeading;                            

    //     // add new position estimate into pose estimator                            
    //     m_Estimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.01, 0.01, 0.001));                     
    //     m_Estimator.addVisionMeasurement(newPose, PoseTimeStamp.get());                              
        
    // }



    // ---------- Apriltag odometry methods ----------
    
    
    // helper function to updates drive wheel odometry - called by periodic()
    private void updateAprilTagOdometry(Limelight camera) {
    
        // get updates from camera
        TagResults = camera.GetJSONResults();
    
        // get time latency from camera
       // double latency = 0.001*RobotContainer.camr.getLatencyContribution();
        
        // if results is not empty and there is a list of apriltags
        if (TagResults!=null && TagResults.targets_Fiducials!=null)
        {
            for (int i=0;i<TagResults.targets_Fiducials.length; ++i)
            {
               
                int tagid = (int)Math.round(TagResults.targets_Fiducials[i].fiducialID);
                
                // if tag is part of coral reef then use it
                if ((tagid >=6 && tagid <=11) || (tagid >=17 && tagid <=22))
                {
                    // set confidence level of apriltag detection
                    // for now assume constant - may be refined later                           
                    m_Estimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 0.1)); 

                    Pose3d pose = TagResults.targets_Fiducials[i].getCameraPose_TargetSpace();
                    double distance = Math.sqrt(pose.getX()*pose.getX() + pose.getZ()*pose.getZ());
                

                    // are we close to apriltag? if so, then use pose estimate
                    if (distance <= 1.5)// was 1.75 
                    {
                        // get field pose from limelight, convert to 2d, then convert to FRC coordinates
                        Pose2d LLpose =  TagResults.targets_Fiducials[i].getRobotPose_FieldSpace().toPose2d();

                        // get angle from gyro 
                        Rotation2d gyroangle = new Rotation2d(Math.toRadians(RobotContainer.gyro.getYawAngle()));

                        Pose2d fieldPose = new Pose2d(LLpose.getX()+8.774176, LLpose.getY()+4.0259, gyroangle);
                    
                        // add in to our position estimator
                        // assume lag time = camera latency + 20ms
                       // m_Estimator.addVisionMeasurement(fieldPose, PoseTimeStamp.get()-latency - 0.02);
                    }
                
                    //m_test.setDouble(RobotContainer.camera.getLatencyContribution());

                } // end if tag belongs to coral reef

        
            } // end for
           
            
            //m_test.setDouble(timeSinceLastTag.get());
            timeSinceLastTag.reset();
        }

    
    }




    // -------------------- Odometry Store/Recall Methods --------------------


    Pose2d m_MemPoints[] = { new Pose2d(0, 0, new Rotation2d(0.0)),
      new Pose2d(0, 0, new Rotation2d(0.0)),
      new Pose2d(0, 0, new Rotation2d(0.0)) };

    /**
    * saves Pose2D coordinate for later recall
    * num = 0 to 2 (three memories available)
    */
    public void RecordPose2d(Pose2d point, int num) {
        if (num < m_MemPoints.length)
            m_MemPoints[num] = point;
    }

    /**
    * recalls Pose2D coordinate previously saved
    * num = 0 to 2 (three memories available)
    */
    public Pose2d RecallPoint(int num) {
        // return saved point. If not in range, simply return 0,0,0 point
        if (num < m_MemPoints.length)
            return m_MemPoints[num];
        else
            return new Pose2d(0, 0, new Rotation2d(0.0));
    }


    // -------------------- Subsystem Shuffleboard Methods --------------------
    
    private GenericEntry m_fieldXPos;
    private GenericEntry m_fieldYPos;
    private GenericEntry m_fieldAngle;
    private GenericEntry m_DWfieldXPos;
    private GenericEntry m_DWfieldYPos;
    private GenericEntry m_DWfieldAngle;
    
    
    private GenericEntry m_detectedtags;
    private GenericEntry m_test;

    /** Initialize subsystem shuffleboard page and controls */
    private void initializeShuffleboard() {
        // Create page in shuffleboard
        ShuffleboardTab Tab = Shuffleboard.getTab("Odometry");
        
        // robot position info
        ShuffleboardLayout l1 = Tab.getLayout("Odometry", BuiltInLayouts.kList);
        l1.withPosition(0, 0);
        l1.withSize(2, 4);
        m_fieldXPos = l1.add("Field X",0.0 ).getEntry();
        m_fieldYPos = l1.add("Field Y",0.0 ).getEntry();
        m_fieldAngle = l1.add("Angle",0.0 ).getEntry();

        // apriltag info
        ShuffleboardLayout l2 = Tab.getLayout("Tag Info", BuiltInLayouts.kList);
        l2.withPosition(2, 0);
        l2.withSize(1, 4);
        m_detectedtags = l2.add("Detections",new String() ).getEntry();
        m_test = l2.add("Test", 0.0).getEntry();
        

         // deadwheel info
         ShuffleboardLayout l3 = Tab.getLayout("Deadwheel Info(Temp)", BuiltInLayouts.kList);
         l3.withPosition(3, 0);
         l3.withSize(2, 4);
         m_DWfieldXPos = l3.add("DW Field X",0.0 ).getEntry();
         m_DWfieldYPos = l3.add("DW Field Y",0.0 ).getEntry();
         m_DWfieldAngle = l3.add("DW Angle",0.0 ).getEntry();

    }

    /** Update subsystem shuffle board page with current odometry values */
    private void updateShuffleboard() {
        Pose2d CurrentPose = getPose2d();
        m_fieldXPos.setDouble(CurrentPose.getX());
        m_fieldYPos.setDouble(CurrentPose.getY());
        m_fieldAngle.setDouble(CurrentPose.getRotation().getDegrees()); 
        
        // temp
        m_DWfieldXPos.setDouble(DWfieldX);
        m_DWfieldYPos.setDouble(DWfieldY);
        m_DWfieldAngle.setDouble(Math.toDegrees(DWfieldAngle)); 

        // show detected tags
        String tags = new String();
        if (TagResults!=null && TagResults.targets_Fiducials!=null)
            for (int i=0;i<TagResults.targets_Fiducials.length; ++i)
                tags=tags+(int)TagResults.targets_Fiducials[i].fiducialID + " ";
        m_detectedtags.setString(tags);
    }
    public void EnableApriltagProcessing(boolean enable){
        TagEnable = enable;
    }


    // ---------- Subsystem Simulation Methods ----------
    

    /** Method called periodically by the scheduler */
    @Override
    public void simulationPeriodic() {

        // drive kinematics
        SwerveDriveKinematics kinematics = RobotContainer.drivesystem.getKinematics();
        
        // swerve drive states
        SwerveModuleState[] states = RobotContainer.drivesystem.getTargetModuleStates();
        
        // get chassis speeds (in robot frame)
        ChassisSpeeds speed = kinematics.toChassisSpeeds(states);
        
        // convert to field relative speeds
        speed = ChassisSpeeds.fromRobotRelativeSpeeds(speed, Rotation2d.fromDegrees(RobotContainer.gyro.getYawAngle()));

        // get current robot pose
        Pose2d currentPose = m_Estimator.getEstimatedPosition();

        // time since last iteration. reset timer for next iteration
        double dt = simTimer.get();
        simTimer.reset();

        // update simulated robot position
        double x = currentPose.getX();
        double y = currentPose.getY();
        double heading = currentPose.getRotation().getRadians();
        
        // integrate speeds to get change in pose
        x += dt*speed.vxMetersPerSecond;
        y += dt*speed.vyMetersPerSecond;
        heading += dt*speed.omegaRadiansPerSecond;
        
        // set new position of robot
        //setPose(new Pose2d (x, y, new Rotation2d(heading)), new Rotation2d(heading)); 
        m_Estimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.001, 0.001, 0.001));                     
        m_Estimator.addVisionMeasurement(new Pose2d (x, y, new Rotation2d(heading)), PoseTimeStamp.get());     
    }



}
