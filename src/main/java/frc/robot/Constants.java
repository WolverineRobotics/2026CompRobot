package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static class DriveConstants {

        // PIDs for drive motor in swerve modules 
        public static final double kDriveP = 0.1; 
        public static final double kDriveI = 0.01; 
        public static final double kDriveD = 0.01; 

        // PIDs for angle motor in swerve modules 
        public static final double kAngleP = 0.1; 
        public static final double kAngleI = 0.01; 
        public static final double kAngleD = 0.01; 

        // PIDs for angle motor in swerve modules 
        public static final double kHeadingP = 0; 
        public static final double kHeadingI = 0; 
        public static final double kHeadingD = 0;

        // CAN IDs for drive motors 
        public static final int frontLeftDriveID = 0; 
        public static final int frontRightDriveID = 1; 
        public static final int backLeftDriveID = 2; 
        public static final int backRightDriveID = 3; 

        // CAN ID for angle motors
        public static final int frontLeftAngleID = 4; 
        public static final int frontRightAngleID = 5; 
        public static final int backLeftAngleID = 6; 
        public static final int backRightAngleID = 7; 

        // Ports of absolute encoders 
        public static final int frontLeftAbsoluteEncoder = 0; 
        public static final int frontRightAbsoluteEncoder = 1; 
        public static final int backLeftAbsoluteEncoder = 2; 
        public static final int backRightAbsoluteEncoder = 3; 

        // Absolute Encoder offsets 
        public static final double frontLeftEncoderOffset = 0;
        public static final double frontRightEncoderOffset = 0;
        public static final double backLeftEncoderOffset = 0;
        public static final double backRightEncoderOffset = 0;

        // Pigeon2 CAN ID 
        public static final int gyroID = 8; 

        // Conversion factor for radians per second --> rotations per minute
        public static final double rpmConversionFactor = (2 * Math.PI) / 60; 

        public static final double wheelRadius = 0.1;

        // Module Locations 
        public static final double xTranslation = Units.inchesToMeters(14);
        public static final double yTranslation = Units.inchesToMeters(14);
        public static final double robotRadius = Math.sqrt(
            xTranslation * xTranslation + 
            yTranslation * yTranslation
        ); 

        public static final Pose2d hubPoseBlue = new Pose2d(
            Units.inchesToMeters(181.56),
            Units.inchesToMeters(158.50), 
            new Rotation2d()        
        ); 

        public static final Pose2d hubPoseRed = new Pose2d(
            Units.inchesToMeters(650.12 - 181.56),
            Units.inchesToMeters(158.50), 
            new Rotation2d()        
        ); 

        public static final double maxSpeed = 5; // m/s 
        public static final double maxAngularVelocity = Math.PI; //rad/s



    }
    
}
