package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    private final SwerveModule topLeftModule; 
    private final SwerveModule topRightModule; 
    private final SwerveModule bottomLeftModule; 
    private final SwerveModule bottomRightModule; 

    public DriveSubsystem() {

        // Declaring Swerve Module objects 
        topLeftModule = new SwerveModule(
            DriveConstants.topLeftDriveID, 
            DriveConstants.topLeftAngleID,
            DriveConstants.topLeftAbsoluteEncoder, 
            DriveConstants.topLeftEncoderOffset
        );

        topRightModule = new SwerveModule(
            DriveConstants.topRightDriveID, 
            DriveConstants.topRightAngleID,
            DriveConstants.topRightAbsoluteEncoder, 
            DriveConstants.topRightEncoderOffset
        );

        bottomLeftModule = new SwerveModule(
            DriveConstants.bottomLeftDriveID, 
            DriveConstants.bottomLeftAngleID,
            DriveConstants.bottomLeftAbsoluteEncoder, 
            DriveConstants.bottomLeftEncoderOffset
        );

        bottomRightModule = new SwerveModule(
            DriveConstants.bottomRightDriveID, 
            DriveConstants.bottomRightAngleID,
            DriveConstants.bottomRightAbsoluteEncoder,
            DriveConstants.bottomRightEncoderOffset
        );
    }
    
}
