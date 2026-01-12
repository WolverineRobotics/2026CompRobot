package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    // Declaring Swerve Module objects
    private final SwerveModule frontLeftModule; 
    private final SwerveModule frontRightModule; 
    private final SwerveModule backLeftModule; 
    private final SwerveModule backRightModule; 

    // Declaring Swerve Kinematics Objects 
    private final SwerveDriveKinematics m_DriveKinematics; 

    // Declaring Gyroscope
    private final Pigeon2 gyro; 

    public DriveSubsystem() {

        // Defining Swerve Module objects 
        frontLeftModule = new SwerveModule(
            DriveConstants.frontLeftDriveID, 
            DriveConstants.frontLeftAngleID,
            DriveConstants.frontLeftAbsoluteEncoder, 
            DriveConstants.frontLeftEncoderOffset
        );

        frontRightModule = new SwerveModule(
            DriveConstants.frontRightDriveID, 
            DriveConstants.frontRightAngleID,
            DriveConstants.frontRightAbsoluteEncoder, 
            DriveConstants.frontRightEncoderOffset
        );

        backLeftModule = new SwerveModule(
            DriveConstants.backLeftDriveID, 
            DriveConstants.backLeftAngleID,
            DriveConstants.backLeftAbsoluteEncoder, 
            DriveConstants.backLeftEncoderOffset
        );

        backRightModule = new SwerveModule(
            DriveConstants.backRightDriveID, 
            DriveConstants.backRightAngleID,
            DriveConstants.backRightAbsoluteEncoder,
            DriveConstants.backRightEncoderOffset
        );

        // Defining Swerve Kinematics 
        m_DriveKinematics = new SwerveDriveKinematics(
            new Translation2d(), 
            new Translation2d(),
            new Translation2d(), 
            new Translation2d()
        );

        // Defining Gyroscope 
        gyro = new Pigeon2(DriveConstants.gyroID); 
    }

    public void drive(double horizontal, double vertical, double rotation) {
        // Generating the nessasery modules states for the given speeds
        SwerveModuleState[] moduleStates = m_DriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(horizontal, vertical, rotation, gyro.getRotation2d())
        ); 

        // Setting each swerve module to the correct state
        frontLeftModule.setState(moduleStates[0]);
        frontRightModule.setState(moduleStates[1]);
        backLeftModule.setState(moduleStates[2]);
        backRightModule.setState(moduleStates[3]);
    }
    
}
