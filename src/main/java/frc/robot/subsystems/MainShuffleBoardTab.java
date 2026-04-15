package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.data.BuildConstants;
/** Contains shuffleboard setup for generic main page not belonging to any subsubsystem
 * Typically used to have robot-level settings and command functions */
public class MainShuffleBoardTab extends SubsystemBase {

    // example autonomous path shuffleboard selection boxes
    // true if selected, false if not
    // <add any other controls here that go on main shufflebard page
    private GenericEntry m_delayTime;
    private SendableChooser<Integer> m_autonomousPath;
    private SendableChooser<Integer> m_StartPosition;


    // Odometry Shuffleboad Entries
    private GenericEntry m_robotX;
    private GenericEntry m_robotY;
    private GenericEntry m_robotAngle;
  
    // field visualization object to display on shuffleboard
    private Field2d m_field = new Field2d();

    // other controls on main page
    private GenericEntry m_timeLeft;
   
    // timeslice timer
    private Timer dt_timer;
    private GenericEntry m_dt;

    // canbus utilization
    private CANBus canbus;
    private GenericEntry m_CANBusload;

    /** Initializes the Shuffleboard
     * Creates the subsystem pages */
    public MainShuffleBoardTab() {

        // add autonomous commands to shuffleboard
        initializeMainShuffleboardPage();


        // create timer and start
        dt_timer = new Timer();
        dt_timer.reset();
        dt_timer.start();

        // used to get canbus stats
        canbus = new CANBus();
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

        // update main page
        // update remaining time in match (rounded to nearest second)
        //m_selectedPath = (Integer)m_autonomousPath.getSelected();
        m_timeLeft.setDouble(Math.round(Timer.getMatchTime()));
        
        
        // set calculation time interval in ms
        m_dt.setDouble(dt_timer.get()*1000.0);
        dt_timer.reset();

        // canbus loading
        m_CANBusload.setDouble(canbus.getStatus().BusUtilization*100.0);

        // update robot position in field widget
        m_field.setRobotPose(RobotContainer.odometry.getPose2d());

        // update odometry
        m_robotX.setDouble(RobotContainer.odometry.getPose2d().getX());
        m_robotY.setDouble(RobotContainer.odometry.getPose2d().getY());
        double angle = RobotContainer.odometry.getPose2d().getRotation().getDegrees();
        if (angle > 0)
        angle = (angle+180.0)%360.0 - 180.0;
        else
        angle = (angle-180.0)%360.0 + 180.0;
        m_robotAngle.setDouble(angle);
    }

    
    /** returns delay for autonomous routines */
    public double getAutoDelay()
    {
        return m_delayTime.getDouble(0.0);
    }

    // returns index of selected auto
    public int getSelectedAutoIndex()
    {
        return m_autonomousPath.getSelected();
    }

    public int getStartPositionIndex()
    {
        return m_StartPosition.getSelected();
    }

    // -------------------- Shuffboard Methods --------------------



    /** Create main shuffleboard page
     * Typically contains autonomous commands and other top-level robot controls*/
    private void initializeMainShuffleboardPage() {

        // Create Main Tab in Shuffleboard
        ShuffleboardTab tab = Shuffleboard.getTab("Main");
        
        // add autonomous commands to page -
        m_autonomousPath = new SendableChooser<Integer>();
        m_autonomousPath.addOption("Do Nothing",0);
        m_autonomousPath.addOption("Shoot Preloads",1);
        m_autonomousPath.addOption("L Straight Auto", 2);
        m_autonomousPath.addOption("R Straight Auto", 3);
        m_autonomousPath.addOption("L Sweep Auto", 4);
        m_autonomousPath.addOption("R Sweep Auto", 5);
        m_autonomousPath.addOption("L Passing Auto", 6);
        m_autonomousPath.addOption("R Passing Auto", 7);
        //m_autonomousPath.addOption("Off Line & Shoot Preloads", 2);
        //m_autonomousPath.addOption("Preloads and Humen Station", 3);;
        m_autonomousPath.setDefaultOption("Do Nothing", 0);

        // add selection box of paths
        tab.add("Auto Selection", m_autonomousPath)
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withPosition(0, 0)
                .withSize(2,1);



        // start position
        m_StartPosition = new SendableChooser<Integer>();
        m_StartPosition .addOption("1-RightBump",0);
        m_StartPosition .addOption("2-LeftBump",1);
        m_StartPosition .addOption("3-RightTrench", 2);
        m_StartPosition .addOption("2-LeftTrench",3);
        m_StartPosition .setDefaultOption("1-RightBump", 0);

        // add selection box of paths
        tab.add("Start Pos'n", m_StartPosition)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(2, 1)
        .withSize(2,1);
        
        // time to delay auto by
        m_delayTime = tab.add("Auto Delay Time", 0)
                        .withWidget(BuiltInWidgets.kNumberSlider)
                        .withPosition(2, 0).
                        withSize(2, 1).
                        withProperties(Map.of("min", 0, "max", 15))
                        .getEntry();
        

        // create controls to display robot position, angle, and gyro angle
        ShuffleboardLayout OdometryInfoLayout = tab.getLayout("Odometry", BuiltInLayouts.kList);
        OdometryInfoLayout.withPosition(0, 1);
        OdometryInfoLayout.withSize(2, 3);
        m_robotX = OdometryInfoLayout.add("X (m)", 0.0).getEntry();
        m_robotY = OdometryInfoLayout.add("Y (m)", 0.0).getEntry();
        m_robotAngle = OdometryInfoLayout.add("Angle(deg)", 0.0).getEntry();
  
        // add field widget 
        tab.add("Field", m_field)
        .withPosition(4, 0)
        .withSize(4, 3);


        // Create System Info Page
        ShuffleboardTab tab2 = Shuffleboard.getTab("Sys Info");
        
        // add match time remaining in autonomous/teleop part of match (seconds)
        ShuffleboardLayout l1 = tab2.getLayout("Stats", BuiltInLayouts.kList);
        l1.withPosition(2, 0);
        l1.withSize(2, 3);
        m_timeLeft = l1.add("TimeLeft(s)", 0.0).getEntry();
        m_dt = l1.add("Calc Interval(ms)", 0.0).getEntry();
        m_CANBusload = l1.add("CAN load(%)", 0.0).getEntry();

        // Uses auto generated constants to put git info on dashboard
        // Only updated once at the beginning
        ShuffleboardLayout BuildInfoLayout = tab2.getLayout("Build Info", BuiltInLayouts.kList);
        BuildInfoLayout.withPosition(0, 0);
        BuildInfoLayout.withSize(2, 3);
        BuildInfoLayout.add("Deployed Branch", BuildConstants.GIT_BRANCH);
        BuildInfoLayout.add("Build Timestamp", BuildConstants.BUILD_DATE);
        BuildInfoLayout.add("Repository", BuildConstants.MAVEN_NAME);    

    }

    
    // -------------------- Field Visulation Methods --------------------


    // draws supplied trajectory on field widget on shuffleboard
    public void setFieldTrajectory(Trajectory trajectory)
    {
        if (trajectory!=null)
            m_field.getObject("Field").setTrajectory(trajectory);
        else
            m_field.getObject("Field").setTrajectory(new Trajectory());
    }

    // remote currently shown field trajectory
    public void deleteFieldTrajectory()
    {
        // remove by sending empty array of poses to field object
        Pose2d poses[] = {};
        m_field.getObject("Field").setPoses(poses);
    }


} // end class MainShuffleBoardPage


