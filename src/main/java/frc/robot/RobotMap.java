package frc.robot;


/**
 * Mapping and creation of hardware on the robot
 */
public class RobotMap {
    
    /**
     * Inner class to hold CAN ID constants.
     */
    public static class CANID {
        
        // CAN IDs for Swerve Cancoders
        public static final int LF_CANCODER = 10;
        public static final int RF_CANCODER = 11;
        public static final int LR_CANCODER = 12;
        public static final int RR_CANCODER  = 13;
        // CAN IDs for Steer Motors
        public static final int LF_STEER_MOTOR = 2;
        public static final int RF_STEER_MOTOR = 4;
        public static final int LR_STEER_MOTOR = 6;
        public static final int RR_STEER_MOTOR = 8;
        // CAN IDs for Drive Motors
        public static final int LF_DRIVE_MOTOR = 3;
        public static final int RF_DRIVE_MOTOR = 5;
        public static final int LR_DRIVE_MOTOR = 7;
        public static final int RR_DRIVE_MOTOR = 9;
        // CAN Ids for Coral Intake Motors 
        
        // CAN ID for CTR Pigeon Gyro
        public static final int PIGEON = 14;
       

    }

    /**
     * Inner class to hold RoboRIO I/O connection constants
     */
  public static class DIO {
        // Deadwheel odometry pods

        // public static final int LEFTENCODER_A = 1;
        // public static final int LEFTENCODER_B = 0;
        // public static final int FRONTENCODER_A = 4;
        // public static final int FRONTENCODER_B = 5;
        // public static final int REARENCODER_A = 3;
        // public static final int REARENCODER_B = 2;
        public static final int photoSensor = 9;

        
    }

    public static class PWMPorts {
        /** PWM Port for led strip */
        

        // PWM port for camera tilting subsystem
        //public static final int CAMERA_SERVO_ID = 1;
    } 

    /**
     * Inner class to hold RoboRIO analog input constants
     */
    public static class AINPorts {
        
        // IDs for NEO Swerve steer position sensors
        public static final int NEO_LF_STEER_SENSOR = 0;
        public static final int NEO_RF_STEER_SENSOR = 1;
        public static final int NEO_LR_STEER_SENSOR = 2;
        public static final int NEO_RR_STEER_SENSOR = 3;

    }

    // game controller port IDs
    public static class GamePadPorts {
        public static final int DriverID = 0;
        public static final int OperatorID = 1;
    }

}