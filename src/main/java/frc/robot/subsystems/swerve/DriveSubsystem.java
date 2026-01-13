package frc.robot.subsystems.swerve;



import java.util.Vector;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
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
            new Translation2d(DriveConstants.xTranslation, DriveConstants.yTranslation), 
            new Translation2d(-DriveConstants.xTranslation, DriveConstants.yTranslation), 
            new Translation2d(DriveConstants.xTranslation, -DriveConstants.yTranslation), 
            new Translation2d(-DriveConstants.xTranslation, -DriveConstants.yTranslation)
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
        SwerveModuleState[] targetStates = getTargetStates(
            new ChassisSpeeds(vertical, horizontal, rotation)
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

    private SwerveModuleState[] getTargetStates(ChassisSpeeds targetSpeed) {

        // Calculating the magnetude of the velocity for translation
        double velocityTranslation = Math.sqrt(
            targetSpeed.vxMetersPerSecond * targetSpeed.vxMetersPerSecond +
            targetSpeed.vyMetersPerSecond * targetSpeed.vyMetersPerSecond
        ); 
        
        // Finding the angle of the velocity Vector
        double thetaTranslation = Math.atan(targetSpeed.vyMetersPerSecond / targetSpeed.vxMetersPerSecond);

        // Applying a correction to get the angle between 0 and 2pi instead of -pi/2 and pi/2
        if (thetaTranslation < 0) {
            thetaTranslation += (2 * Math.PI); 
        } 

        if (targetSpeed.vxMetersPerSecond < 0) {
            if (targetSpeed.vyMetersPerSecond < 0) {
                thetaTranslation += (Math.PI); 
            }

            else {
                thetaTranslation -= Math.PI; 
            }
        }

        double velocityRotation = (targetSpeed.omegaRadiansPerSecond * DriveConstants.robotRadius) / 4; 
        double frontLeftTheta = (3 * Math.PI) / 4; 
        double frontRightTheta = (Math.PI) / 4; 
        double backLeftTheta = (5 * Math.PI) / 4; 
        double backRightTheta = (7 * Math.PI) / 4; 

        double frontLeftVelocity = Math.sqrt(
            (targetSpeed.vxMetersPerSecond + velocityRotation * Math.sin(frontLeftTheta)) * (targetSpeed.vxMetersPerSecond + velocityRotation * Math.sin(frontLeftTheta)) + 
            (targetSpeed.vyMetersPerSecond  + velocityRotation * Math.cos(frontLeftTheta)) * (targetSpeed.vxMetersPerSecond + velocityRotation * Math.cos(frontLeftTheta))
        ); 

        double frontRightVelocity = Math.sqrt(
            (targetSpeed.vxMetersPerSecond + velocityRotation * Math.cos(frontLeftTheta)) * (targetSpeed.vxMetersPerSecond + velocityRotation * Math.cos(frontLeftTheta)) + 
            (targetSpeed.vyMetersPerSecond  + velocityRotation * Math.sin(frontLeftTheta)) * (targetSpeed.vxMetersPerSecond + velocityRotation * Math.sin(frontLeftTheta))
        ); 

        double backLeftVelocity = Math.sqrt(
            (targetSpeed.vxMetersPerSecond + velocityRotation * Math.cos(frontLeftTheta)) * (targetSpeed.vxMetersPerSecond + velocityRotation * Math.cos(frontLeftTheta)) + 
            (targetSpeed.vyMetersPerSecond  + velocityRotation * Math.sin(frontLeftTheta)) * (targetSpeed.vxMetersPerSecond + velocityRotation * Math.sin(frontLeftTheta))
        ); 

        double backRightVelocity = Math.sqrt(
            (targetSpeed.vxMetersPerSecond + velocityRotation * Math.sin(frontLeftTheta)) * (targetSpeed.vxMetersPerSecond + velocityRotation * Math.sin(frontLeftTheta)) + 
            (targetSpeed.vyMetersPerSecond  + velocityRotation * Math.cos(frontLeftTheta)) * (targetSpeed.vxMetersPerSecond + velocityRotation * Math.sin(frontLeftTheta))
        ); 
        

        return new SwerveModuleState[] {
            new SwerveModuleState(frontLeftVelocity, new Rotation2d(frontLeftTheta)),
            new SwerveModuleState(frontRightVelocity, new Rotation2d(frontRightTheta)),
            new SwerveModuleState(backLeftVelocity, new Rotation2d(backLeftTheta)),
            new SwerveModuleState(backRightVelocity, new Rotation2d(backRightTheta)),
        };
    }
    
}
