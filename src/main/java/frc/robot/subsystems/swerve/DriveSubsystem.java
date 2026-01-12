package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DefaultDriveCommand;

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

    // Declaring publisher for module states
    private final StructArrayPublisher<SwerveModuleState> moduleStatesPublisher; 
    private final StructArrayPublisher<SwerveModuleState> targetStatesPublisher; 

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
            new Translation2d(Units.inchesToMeters(14), Units.inchesToMeters(14)), 
            new Translation2d(Units.inchesToMeters(-14), Units.inchesToMeters(14)), 
            new Translation2d(Units.inchesToMeters(14), Units.inchesToMeters(-14)), 
            new Translation2d(Units.inchesToMeters(-14), Units.inchesToMeters(-14)) 
         
        );

        // Defining Gyroscope 
        gyro = new Pigeon2(DriveConstants.gyroID); 

        // Setting default command
        this.setDefaultCommand(new DefaultDriveCommand(this));

        // Defing module state publisher 
        moduleStatesPublisher = NetworkTableInstance.getDefault().getStructArrayTopic(
            "Module States", SwerveModuleState.struct).publish();

        targetStatesPublisher = NetworkTableInstance.getDefault().getStructArrayTopic(
            "Target States", SwerveModuleState.struct).publish();
    }

    public void drive(double vertical, double horizontal, double rotation) {
        // Generating the nessasery modules states for the given speeds
        SwerveModuleState[] targetStates = m_DriveKinematics.toSwerveModuleStates(
            new ChassisSpeeds(vertical, horizontal, -rotation)
        ); 

        targetStatesPublisher.set(
            new SwerveModuleState[] {
                targetStates[0], 
                targetStates[1], 
                targetStates[2], 
                targetStates[3]
            }
        );
        
        // Setting each swerve module to the correct state
        frontLeftModule.setState(targetStates[0]);
        frontRightModule.setState(targetStates[1]);
        backLeftModule.setState(targetStates[2]);
        backRightModule.setState(targetStates[3]);
    }


    @Override
    public void periodic() {
       
        // Publishing the current modules states 
        moduleStatesPublisher.set(
        new SwerveModuleState[] {
            frontLeftModule.getModuleState(), 
            frontRightModule.getModuleState(), 
            backLeftModule.getModuleState(), 
            backRightModule.getModuleState()
        }
       );

       SmartDashboard.putNumber("Current Angle", frontLeftModule.getAbsoluteAngle().getDegrees()); 
       SmartDashboard.putNumber("Current Speed", frontLeftModule.getDriveVelocity()); 
    }
    
}
