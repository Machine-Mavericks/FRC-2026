// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.annotation.JsonProperty;



public class Limelight extends SubsystemBase {
 
    // network table to communicate to camera with
    private NetworkTable m_table;

    // true if camera is used to detect apriltags
    private boolean m_FiducialEnable;
    
    
    /** Creates a new Limelight.
     * Input: String containing name of limelight (defined in the camera) */
    public Limelight(String name, boolean FiducialEnable) {
      ConstructLimelight(name, FiducialEnable); }
    public Limelight(String name) {
      ConstructLimelight(name, false); }

    
    // construct limelight subsystem
    private void ConstructLimelight(String name, boolean FiducialEnable) {
        
        // record fiducial enable
        m_FiducialEnable = FiducialEnable;
        
        // set pointer to limelight network table
        m_table = NetworkTableInstance.getDefault().getTable("limelight-"+name);
        
        // initialize camera to use LED mode set in the current pipeline setup
        m_table.getEntry("ledMode").setNumber(0);

        // set camera streaming mode - primary and secondary cameras are placed
        // side-by-side
        m_table.getEntry("stream").setNumber(0);

        // set initial pipeline to 0
        setPipeline(0);
    
        // create shuffleboard page
        initializeShuffleboard(name);
    }


    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        // update shuffleboard
        updateShuffleboard();
    }
  

    // ---------- Camera Control Functions ----------
  
    /** set camera's current pipeline: 0 to 9 */
    public void setPipeline(int num) {
        if (num >= 0 && num <= 9)
        m_table.getEntry("pipeline").setNumber(num);
    }
  
    /** returns camera's current pipeline: 0 to 9 */
    public Double getPipeline() {
        return m_table.getEntry("getPipe").getDouble(0);
    }

    /** get camera heartbeat - increases once per frame, resets at 2billion */
    public double getHeartbeat() {
        return m_table.getEntry("hb").getDouble(0);
    }
  

    // --------------- General Target Functions ---------------------
  
    /**
     * get horitaonal angle from center of camera view to center of target
     * returns -27 to +27 degrees (-29.8 to 29.8 for LL2)
     */
    public float getHorizontalTargetOffsetAngle() {
        return m_table.getEntry("tx").getFloat(0);
    }
    // tx -5.9 
    // -5.3
  
    /**
     * get vertical angle from center of camera view to center of target
     * returns -20 to +20 degrees (-24.85 to 24.85 for LL2)
     */
    public float getVerticalTargetOffsetAngle() {
        return m_table.getEntry("ty").getFloat(0);
    }
  
    // get target detection time latency
    public double getLatencyContribution() {
        return m_table.getEntry("tl").getDouble(0);
    }
  
    /** get whether target is currently detected or not, returns 0 or 1 */ 
    public boolean isTargetPresent() {
        return (m_table.getEntry("tv").getDouble(0.0)>0.05);
    }
  
    /** get target area atributes - 0 to 100% of image */
    public double getTargetArea() {
        return m_table.getEntry("ta").getDouble(0);
    }

    
    // -------------------- General Apriltag Functions --------------------
  
    /** Get primary april tag id */
    public double getPrimAprilTagID () {
        return m_table.getEntry("tid").getDouble(0);
    }

    // set primary tag Id for txty targertting 
    public void SetPriorityTagID(int tagnum){
        m_table.getEntry("priorityid").setInteger(tagnum);
    }

    
    /** Set list of tag IDs to be used for localization.
     * Tags not in this list will be ignored */
    public void SetFiducialIDFiltersOverride(int[] validIDs) {
        double[] validIDsDouble = new double[validIDs.length];
        for (int i=0; i<validIDs.length;++i)
            validIDsDouble[i]=validIDs[i];
        m_table.getEntry("fiducial_id_filters_set").setDoubleArray(validIDsDouble);
    }

    /** Set apriltag downscaling factor.
     * Valid values, 0(pipeline control), 1 (no downscaling),2, 3, 4*/
    public void SetFiducialDownscalingOverride(int downscale)
    {
        if (downscale >= 0 && downscale<=4)
            m_table.getEntry("fiducial_downscale_set").setDouble(downscale);
    } 
    
    /* get Robot Metatag pose3d */
    public Pose3d getBotPose() {
        double[] vector = m_table.getEntry("botpose_wpiblue").getDoubleArray(new double[]{});  
        return toPose3D(vector);
    }
    
    /* get Robot MegaTag2 pose3d */
    public Pose3d getBotPoseOrb() {
        double[] vector = m_table.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[]{});  
        return toPose3D(vector);
    }

    /* get Robot Megatag details*/
    public PoseEstimate getBotPoseDetail() {
        return getBotPoseEstimate("botpose_wpiblue", false);
    }

    /* get Robot MegaTag2 pose3d */
    public PoseEstimate getBotPoseOrbDetail() {
        return getBotPoseEstimate("botpose_orb_wpiblue", true);
    }


    /** Represents a 3D Pose Estimate.*/
    public static class PoseEstimate {
        public Pose3d pose;
        public double timestampSeconds;
        public double latency;
        public int tagCount;
        public double tagSpan;
        public double avgTagDist;
        public double avgTagArea;
        public RawFiducial[] rawFiducials; 
        public boolean isMegaTag2;

        // Instantiates a PoseEstimate object with default values
        public PoseEstimate() {
            this.pose = new Pose3d();
            this.timestampSeconds = 0;
            this.latency = 0;
            this.tagCount = 0;
            this.tagSpan = 0;
            this.avgTagDist = 0;
            this.avgTagArea = 0;
            this.rawFiducials = new RawFiducial[]{};
            this.isMegaTag2 = false;
        }

        public PoseEstimate(Pose3d pose, double timestampSeconds, double latency, 
            int tagCount, double tagSpan, double avgTagDist, 
            double avgTagArea, RawFiducial[] rawFiducials, boolean isMegaTag2) {
            this.pose = pose;
            this.timestampSeconds = timestampSeconds;
            this.latency = latency;
            this.tagCount = tagCount;
            this.tagSpan = tagSpan;
            this.avgTagDist = avgTagDist;
            this.avgTagArea = avgTagArea;
            this.rawFiducials = rawFiducials;
            this.isMegaTag2 = isMegaTag2;
        }
    }


    private PoseEstimate getBotPoseEstimate(String entryName, boolean isMegaTag2) {
        
        DoubleArrayEntry poseEntry = m_table.getDoubleArrayTopic(entryName).getEntry(new double[0]);
        
        TimestampedDoubleArray tsValue = poseEntry.getAtomic();
        double[] poseArray = tsValue.value;
        long timestamp = tsValue.timestamp;
        
        if (poseArray.length == 0) {
            // Handle the case where no data is available
            return null; // or some default PoseEstimate
        }
    
        Pose3d pose = toPose3D(poseArray);
        double latency = extractArrayEntry(poseArray, 6);
        int tagCount = (int)extractArrayEntry(poseArray, 7);
        double tagSpan = extractArrayEntry(poseArray, 8);
        double tagDist = extractArrayEntry(poseArray, 9);
        double tagArea = extractArrayEntry(poseArray, 10);
        
        // Convert server timestamp from microseconds to seconds and adjust for latency
        double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);
    
        RawFiducial[] rawFiducials = new RawFiducial[tagCount];
        int valsPerFiducial = 7;
        int expectedTotalVals = 11 + valsPerFiducial * tagCount;
    
        if (poseArray.length != expectedTotalVals) {
            // Don't populate fiducials
        } else {
            for(int i = 0; i < tagCount; i++) {
                int baseIndex = 11 + (i * valsPerFiducial);
                int id = (int)poseArray[baseIndex];
                double txnc = poseArray[baseIndex + 1];
                double tync = poseArray[baseIndex + 2];
                double ta = poseArray[baseIndex + 3];
                double distToCamera = poseArray[baseIndex + 4];
                double distToRobot = poseArray[baseIndex + 5];
                double ambiguity = poseArray[baseIndex + 6];
                rawFiducials[i] = new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
            }
        }
    
        return new PoseEstimate(pose, adjustedTimestamp, latency, tagCount, tagSpan, tagDist, tagArea, rawFiducials, isMegaTag2);
    }



    // -------------------- Raw Fiducial Information - (does not use Json parsing) --------------------

    // raw data availalbe for each tag detected
    public static class RawFiducial {
        public int id = 0;
        public double txnc = 0;
        public double tync = 0;
        public double ta = 0;
        public double distToCamera = 0;
        public double distToRobot = 0;
        public double ambiguity = 0;

        public RawFiducial(int id, double txnc, double tync, double ta, double distToCamera, double distToRobot, double ambiguity) {
            this.id = id;
            this.txnc = txnc;
            this.tync = tync;
            this.ta = ta;
            this.distToCamera = distToCamera;
            this.distToRobot = distToRobot;
            this.ambiguity = ambiguity;
        }
    }

    private static double extractArrayEntry(double[] inData, int position){
        if(inData.length < position+1)
        {
            return 0;
        }
        return inData[position];
    }

    // function return rawFiducial information from camera
    public RawFiducial[] getRawFiducials() {

        // get raw fidicual data
        double[] rawFiducialArray = m_table.getEntry("rawfiducials").getDoubleArray(new double[]{});

        int valsPerEntry = 7;
        if (rawFiducialArray.length % valsPerEntry != 0) {
            return new RawFiducial[0];
        }
    
        int numFiducials = rawFiducialArray.length / valsPerEntry;
        RawFiducial[] rawFiducials = new RawFiducial[numFiducials];
    
        for (int i = 0; i < numFiducials; i++) {
            int baseIndex = i * valsPerEntry;
            int id = (int) extractArrayEntry(rawFiducialArray, baseIndex);
            double txnc = extractArrayEntry(rawFiducialArray, baseIndex + 1);
            double tync = extractArrayEntry(rawFiducialArray, baseIndex + 2);
            double ta = extractArrayEntry(rawFiducialArray, baseIndex + 3);
            double distToCamera = extractArrayEntry(rawFiducialArray, baseIndex + 4);
            double distToRobot = extractArrayEntry(rawFiducialArray, baseIndex + 5);
            double ambiguity = extractArrayEntry(rawFiducialArray, baseIndex + 6);
            
            rawFiducials[i] = new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
        }
    
        return rawFiducials;
    }




    // -------------------- Fiducial Classes for JSON File Access and Parsing --------------------


    // gets JSON fiducial target into from camera
    private static ObjectMapper mapper = new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);;
    public LimelightResults GetJSONResults ()
    {
    LimelightResults results = new LimelightResults();
    
    // get json from camera
    String string = m_table.getEntry("json").getString("");
    
    try {
        results = mapper.readValue(string, LimelightResults.class);
    } catch (JsonProcessingException e) {
        //System.err.println("lljson error: " + e.getMessage());
    }

    return results;
    }

    // structure containing JSON results
    public static class LimelightResults {
    
        @JsonProperty("pID")
        public double pipelineID;

        @JsonProperty("botpose")
        public double[] botpose;

        @JsonProperty("Fiducial")
        public LimelightTarget_Fiducial[] targets_Fiducials;

        @JsonProperty("botpose_tagcount")
        public double botpose_tagcount;

        public Pose3d getBotPose()
        { return toPose3D(botpose); }

        public LimelightResults() {
            botpose = new double[6];  
            targets_Fiducials = new LimelightTarget_Fiducial[0];
            botpose_tagcount=0;
        }

    }

    // camera information about each fiducial target
    public static class LimelightTarget_Fiducial {

        @JsonProperty("fID")
        public double fiducialID;

        @JsonProperty("fam")
        public String fiducialFamily;

        @JsonProperty("t6c_ts")
        public double[] cameraPose_TargetSpace;

        @JsonProperty("t6r_fs")
        public double[] robotPose_FieldSpace;

        @JsonProperty("t6r_ts")
        public double[] robotPose_TargetSpace;

        @JsonProperty("t6t_cs")
        public double[] targetPose_CameraSpace;

        @JsonProperty("t6t_rs")
        public double[] targetPose_RobotSpace;

        @JsonProperty("ta")
        public double ta;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("typ")
        public double ty_pixels;

        @JsonProperty("ts")
        public double ts;
    
        public Pose3d getCameraPose_TargetSpace()
        { return toPose3D(cameraPose_TargetSpace); }
        public Pose3d getRobotPose_FieldSpace()
        { return toPose3D(robotPose_FieldSpace); }
        public Pose3d getRobotPose_TargetSpace()
        { return toPose3D(robotPose_TargetSpace); }
        public Pose3d getTargetPose_CameraSpace()
        { return toPose3D(targetPose_CameraSpace); }
        public Pose3d getTargetPose_RobotSpace()
        { return toPose3D(targetPose_RobotSpace);}

        public LimelightTarget_Fiducial() {
            cameraPose_TargetSpace = new double[6];
            robotPose_FieldSpace = new double[6];
            robotPose_TargetSpace = new double[6];
            targetPose_CameraSpace = new double[6];
            targetPose_RobotSpace = new double[6];
            fiducialID = -1;
        }
    }


    // -------------------- Helper Functions --------------------

    // helper to convert array of values to a pose3d
    private static Pose3d toPose3D(double[] vector) {
        // if vector is valid (has 6 numbers in it) go ahead and record data in structure
        if (vector.length<6)
            return new Pose3d();
        else
            return new Pose3d(
            new Translation3d(vector[0], vector[1], vector[2]),
            new Rotation3d(Units.degreesToRadians(vector[3]), Units.degreesToRadians(vector[4]),
                        Units.degreesToRadians(vector[5])));
    }



    // -------------------- Subsystem Shuffleboard Methods --------------------

    // subsystem shuffleboard controls
    private GenericEntry m_Pipeline;
    private GenericEntry m_TargetPresent;
    private GenericEntry m_AprilTagID;
    private GenericEntry m_AngleX;
    private GenericEntry m_AngleY;
    private GenericEntry m_Area;
    private GenericEntry m_BotPose[] = new GenericEntry[6];


    /** Initialize subsystem shuffleboard page and controls */
    private void initializeShuffleboard(String name) {
        // Create odometry page in shuffleboard
        ShuffleboardTab Tab = Shuffleboard.getTab("Limelight: "+name);

        // camera pipeline number
        m_Pipeline = Tab.add("Pipeline", 0)
                        .withPosition(0,0).getEntry();

        // does camera detect target
        m_TargetPresent = Tab.add("Target Present", false).withPosition(0,1).getEntry();

        // april tag target id
        m_AprilTagID = Tab.add("AprilTag Target ID", 0).withPosition(0,2).getEntry();

        // camera target information
        ShuffleboardLayout l1 = Tab.getLayout("Target", BuiltInLayouts.kList);
        l1.withPosition(2, 0);
        l1.withSize(1, 4);
        m_AngleX = l1.add("AngleX", 0.0).getEntry();
        m_AngleY = l1.add("AngleY", 0.0).getEntry();
        m_Area = l1.add("Area", 0.0).getEntry(); 
        


        // only show Fiducial information if requested
        if (m_FiducialEnable) {
            ShuffleboardLayout l3 = Tab.getLayout("BotPose", BuiltInLayouts.kList);
            l3.withPosition(4,0);
            l3.withSize(1,4);
            m_BotPose[0] = l3.add("x", 0.0).getEntry();
            m_BotPose[1] = l3.add("y", 0.0).getEntry(); 
            m_BotPose[2] = l3.add("z", 0.0).getEntry(); 
            m_BotPose[3] = l3.add("rx", 0.0).getEntry(); 
            m_BotPose[4] = l3.add("ry", 0.0).getEntry(); 
            m_BotPose[5] = l3.add("rz", 0.0).getEntry();
            
            // temporary for testing 
            // april tag target id
            // m_Test1 = Tab.add("Num Targets", 0).withPosition(0,3).getEntry();
            // m_Test2 = Tab.add("Target ID", 0).withPosition(0,4).getEntry();
            // end temp ////////
        }

    }


    /** Update subsystem shuffle board page with current odometry values */
    private void updateShuffleboard() {
        
        // update camera pipeline and target detected indicator
        m_Pipeline.setDouble(getPipeline());
        m_TargetPresent.setBoolean(isTargetPresent());
        m_AprilTagID.setDouble(getPrimAprilTagID());

        // update angles to center of target
        m_AngleX.setDouble(getHorizontalTargetOffsetAngle());
        m_AngleY.setDouble(getVerticalTargetOffsetAngle());
        m_Area.setDouble(getTargetArea());


        // only show Fiducial information if requested
        if (m_FiducialEnable) {
            // update robot position in field space
            Pose3d pose = getBotPose();
            m_BotPose[0].setDouble(pose.getTranslation().getX());
            m_BotPose[1].setDouble(pose.getTranslation().getY());
            m_BotPose[2].setDouble(pose.getTranslation().getZ());
            m_BotPose[3].setDouble(Units.radiansToDegrees(pose.getRotation().getX()));
            m_BotPose[4].setDouble(Units.radiansToDegrees(pose.getRotation().getY()));
            m_BotPose[5].setDouble(Units.radiansToDegrees(pose.getRotation().getZ()));

            // temporary for testing purposes
            //LimelightResults results = GetJSONResults();
            //m_Test1.setDouble(results.targetingResults.targets_Fiducials.length);
            //m_Test2.setDouble(results.targetingResults.targets_Fiducials[0].fiducialID);
        }
    }



} // end class LImelight