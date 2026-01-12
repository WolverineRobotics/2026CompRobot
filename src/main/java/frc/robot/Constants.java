package frc.robot;

public class Constants {

    public static class DriveConstants {

        // PIDs for drive motor in swerve modules 
        public static final double kDriveP = 0; 
        public static final double kDriveI = 0; 
        public static final double kDriveD = 0; 

        // PIDs for angle motor in swerve modules 
        public static final double kAngleP = 0; 
        public static final double kAngleI = 0; 
        public static final double kAngleD = 0; 

        // CAN IDs for drive motors 
        public static final int frontLeftDriveID = 0; 
        public static final int frontRightDriveID = 0; 
        public static final int backLeftDriveID = 0; 
        public static final int backRightDriveID = 0; 

        // CAN ID for angle motors
        public static final int frontLeftAngleID = 0; 
        public static final int frontRightAngleID = 0; 
        public static final int backLeftAngleID = 0; 
        public static final int backRightAngleID = 0; 

        // Ports of absolute encoders 
        public static final int frontLeftAbsoluteEncoder = 0; 
        public static final int frontRightAbsoluteEncoder = 0; 
        public static final int backLeftAbsoluteEncoder = 0; 
        public static final int backRightAbsoluteEncoder = 0; 

        // Absolute Encoder offsets 
        public static final double frontLeftEncoderOffset = 0;
        public static final double frontRightEncoderOffset = 0;
        public static final double backLeftEncoderOffset = 0;
        public static final double backRightEncoderOffset = 0;

        // Pigeon2 CAN ID 
        public static final int gyroID = 0; 

    }
    
}
