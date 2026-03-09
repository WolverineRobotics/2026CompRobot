package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static class DriveConstants {

        // PIDs for drive motor in swerve modules 
        public static final double kDriveP = 0.00025; 
        public static final double kDriveI = 0.0; 
        public static final double kDriveD = 0.0000001; 

        // PIDs for angle motor in swerve modules 
        public static final double kAngleP = 0.01; 
        public static final double kAngleI = 0.0; 
        public static final double kAngleD = 0.0001; 

        // PIDs for angle motor in swerve modules 
        public static final double kHeadingP = 0; 
        public static final double kHeadingI = 0; 
        public static final double kHeadingD = 0;

        // CAN IDs for drive motors all odd numbers starting from front left 
        public static final int frontLeftDriveID = 11; 
        public static final int frontRightDriveID = 13; 
        public static final int backLeftDriveID = 15; 
        public static final int backRightDriveID = 17; 

        // CAN ID for angle motors all even number starting from front right
        public static final int frontLeftAngleID = 12; 
        public static final int frontRightAngleID = 14; 
        public static final int backLeftAngleID = 16; 
        public static final int backRightAngleID = 18; 

        // Ports of absolute encoders 
        public static final int frontLeftAbsoluteEncoder = 0; 
        public static final int frontRightAbsoluteEncoder = 2; 
        public static final int backLeftAbsoluteEncoder = 1; 
        public static final int backRightAbsoluteEncoder = 3; 

        // Absolute Encoder offsets 
        public static final double frontLeftEncoderOffset = 95.8;
        public static final double frontRightEncoderOffset = 306;
        public static final double backLeftEncoderOffset = 233;
        public static final double backRightEncoderOffset = 131;

        // Absolute Encoder Inversion
        public static final boolean frontLeftInverted = true; 
        public static final boolean frontRightInverted = true; 
        public static final boolean backLeftInverted = true; 
        public static final boolean backRightInverted = true; 

        // Pigeon2 CAN ID 
        public static final int gyroID = 10; 

        // Conversion factor for radians per second --> rotations per minute
        public static final double rpmConversionFactor = 60 / (2 * Math.PI) ; 

        // Drive PID Scaling Factor 
        public static final double drivePIDScaling = 1.65289; 

        public static final double wheelRadius = Units.inchesToMeters(2);

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

        public static final double maxSpeed = 1; // m/s 
        public static final double maxAngularVelocity = Math.PI; //rad/s

        public static final double reverseTolerence = 2.5; //degrees



    }
    
}
