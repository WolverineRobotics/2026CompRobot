package frc.robot;

public class Constants {

    public static class DriveConstants {

        // PIDs for drive motor in swerve modules 
        public static double kDriveP = 0; 
        public static double kDriveI = 0; 
        public static double kDriveD = 0; 

        // PIDs for angle motor in swerve modules 
        public static double kAngleP; 
        public static double kAngleI; 
        public static double kAngleD; 

        // CAN IDs for drive motors 
        public static int topLeftDriveID; 
        public static int topRightDriveID; 
        public static int bottomLeftDriveID; 
        public static int bottomRightDriveID; 

        // CAN ID for angle motors
        public static int topLeftAngleID; 
        public static int topRightAngleID; 
        public static int bottomLeftAngleID; 
        public static int bottomRightAngleID; 

        // Ports of absolute encoders 
        public static int topLeftAbsoluteEncoder; 
        public static int topRightAbsoluteEncoder; 
        public static int bottomLeftAbsoluteEncoder; 
        public static int bottomRightAbsoluteEncoder; 

        // Absolute Encoder offsets 
        public static double topLeftEncoderOffset = 0;
        public static double topRightEncoderOffset = 0;
        public static double bottomLeftEncoderOffset = 0;
        public static double bottomRightEncoderOffset = 0;

    }
    
}
