package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

public class Constants {
    public static class ShooterConstants {

        public static final int flywheelCANID = 31; 
        public static final int indexerCANID = 32; 

        public static final double flyWheelKp = 0.0; 
        public static final double flyWheelKi = 0.0000; 
        public static final double flyWheelKd = 0.0000000; 

        public static final double flyWheelKs = 0.125; 
        public static final double flyWheelKv = 0.001792021; 
 

        // Radians per Second 
        public static final double maxFlywheelSpeed = 30; 

        // Motor Percentage
        public static final double defaultIndexerSpeed = -0.8; 

        public static final int indexerCurrentLimit = 30; 

        public static final double flywheelSpeed = 2850; 
        public static final double flywheelFunnelSpeed = 4250; 


        public static final double hubHeight = Units.inchesToMeters(70); 
        public static final double shooterAngle = Units.degreesToRadians(60); 
        public static final double flywheelRadius = Units.inchesToMeters(4);

        public static final boolean flywheelInverted = true; 
        public static final double shootDisplacement = 0.23; 
        public static final double shooterHeight = Units.inchesToMeters(20);

        public static final int flywheelCurrentLimit = 40; 

        

    }

    public static class DriveConstants {

        // PIDs for drive motor in swerve modules 
        public static final double kDriveP = 0.000295; 
        public static final double kDriveI = 0.00000; 
        public static final double kDriveD = 0.0000025; 

        public static final double driveScaling = 1.66; 

        // PIDs for angle motor in swerve modules 
        public static final double kAngleP = 0.01; 
        public static final double kAngleI = 0.0; 
        public static final double kAngleD = 0.0001; 

        // PIDs for hub alignment command
        public static final double kHeadingP = 0; 
        public static final double kHeadingI = 0; 
        public static final double kHeadingD = 0;

        // CAN IDs for drive motors 
        public static final int frontRightDriveID = 11; 
        public static final int frontLeftDriveID = 13;        
        public static final int backRightDriveID = 15; 
        public static final int backLeftDriveID = 17; 
       

        // CAN ID for angle motors
        public static final int frontRightAngleID = 12; 
        public static final int frontLeftAngleID = 14; 
        public static final int backRightAngleID = 16; 
        public static final int backLeftAngleID = 18; 
       

        // Ports of absolute encoders 
        public static final int frontRightAbsoluteEncoder = 2; 
        public static final int frontLeftAbsoluteEncoder = 0; 
        public static final int backRightAbsoluteEncoder = 3; 
        public static final int backLeftAbsoluteEncoder = 1; 

        // Drive motor Inversions
        public static final boolean frontRightDriveInverted = true;
        public static final boolean frontLeftDriveInverted = true;
        public static final boolean backRightDriveInverted = true;
        public static final boolean backLeftDriveInverted = true;

        // Absolute Encoder offsets 
        public static final double frontRightEncoderOffset = -234;
        public static final double frontLeftEncoderOffset = -137;
        public static final double backRightEncoderOffset = -206;
        public static final double backLeftEncoderOffset = -135;

        // Absolute Encoder Inversion
        public static final boolean frontLeftInverted = true; 
        public static final boolean frontRightInverted = true; 
        public static final boolean backLeftInverted = true; 
        public static final boolean backRightInverted = true; 

        // Pigeon2 CAN ID 
        public static final int gyroID = 10; 

        // Conversion factor for radians per second --> rotations per minute
        public static final double rpmConversionFactor = 60 / (2 * Math.PI) ; 

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

        public static final double maxSpeed = 20 * driveScaling; // m/s 
        public static final double maxAngularVelocity = (8 * Math.PI); //rad/s

        public static final double reverseTolerence = 5; //degrees

        public static final double maxShootingDistance = 4.2; 
        public static final double minShootingDistance = 3.7; 

        public static final int driveCurrentLimit = 40; 
        public static final int angleCurrentLimit = 40; 




    }
    
    public static class IntakeConstants{
        //Ids for intake + limit
        public static final int intakeMotorID = 23; 
        public static final int leftPivotID = 21; 
        public static final int rightPivotID = 22; 

        public static final double stressedIntakeCurrentDraw = 22; 

        //Base Intake
        public static final double intakeSpeed = 0.45; //temp 

        public static final double pivotSpeed = -0.3; 

        public static final boolean leftPivotInverted = false; 
        public static final boolean rightPivotInverted = false; 

        public static final int pivotCurrentLimit = 40; 
        public static final int rollerCurrentLimit = 40; 


    }


}