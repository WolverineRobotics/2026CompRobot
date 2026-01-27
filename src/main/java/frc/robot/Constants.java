package frc.robot;

import edu.wpi.first.math.util.Units;

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
        public static final int frontLeftDriveID = 31; 
        public static final int frontRightDriveID = 15; 
        public static final int backLeftDriveID = 14; 
        public static final int backRightDriveID = 21; 

        // CAN ID for angle motors
        public static final int frontLeftAngleID = 23; 
        public static final int frontRightAngleID = 12; 
        public static final int backLeftAngleID = 18; 
        public static final int backRightAngleID = 17; 

        // Ports of absolute encoders 
        public static final int frontLeftAbsoluteEncoder = 0; 
        public static final int frontRightAbsoluteEncoder = 2; 
        public static final int backLeftAbsoluteEncoder = 1; 
        public static final int backRightAbsoluteEncoder = 3; 

        // Absolute Encoder offsets 
        public static final double frontLeftEncoderOffset = 0.32;
        public static final double frontRightEncoderOffset = 0.87;
        public static final double backLeftEncoderOffset = 0.68;
        public static final double backRightEncoderOffset = 0.36;

        // Pigeon2 CAN ID 
        public static final int gyroID = 2; 

        // Conversion factor for radians per second --> rotations per minute
        public static final double rpmConversionFactor = (2 * Math.PI) / 60; 

        public static final double wheelRadius = Units.inchesToMeters(2);

        // Module Locations 
        public static final double xTranslation = Units.inchesToMeters(14);
        public static final double yTranslation = Units.inchesToMeters(14);
        public static final double robotRadius = Math.sqrt(
            xTranslation * xTranslation + 
            yTranslation * yTranslation
        ); 

    }
    
}
