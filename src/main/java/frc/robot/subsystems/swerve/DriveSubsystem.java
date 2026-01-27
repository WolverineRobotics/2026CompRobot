package frc.robot.subsystems.swerve;



import java.util.Vector;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
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

    // Declaring Swerve Kinematics and Odometery Objects 
    private final SwerveDriveKinematics m_DriveKinematics; 
    private final SwerveDriveOdometry m_DriveOdometry; 

    // Declaring Gyroscope
    private final Pigeon2 gyro;

    // Declaring publisher for module states
    private final StructArrayPublisher<SwerveModuleState> moduleStatesPublisher; 
    private final StructArrayPublisher<SwerveModuleState> targetStatesPublisher; 

    // Declaring publisher for Robot Pose 
    private final StructPublisher<Pose2d> robotPosePublisher; 

    
    /**
     * Constructs the Drive subsystem 
     */
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

        // Defining Gyroscope 
        gyro = new Pigeon2(DriveConstants.gyroID); 


        // Defining Swerve Kinematics and Odometery 
        m_DriveKinematics = new SwerveDriveKinematics(
            new Translation2d(DriveConstants.xTranslation, DriveConstants.yTranslation),  // Front Left
            new Translation2d(DriveConstants.xTranslation, -DriveConstants.yTranslation), // Back Left 
            new Translation2d(-DriveConstants.xTranslation, DriveConstants.yTranslation), // Front Right 
            new Translation2d(-DriveConstants.xTranslation, -DriveConstants.yTranslation) // Back Right
        );

        m_DriveOdometry = new SwerveDriveOdometry(m_DriveKinematics, gyro.getRotation2d(),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(), 
                backLeftModule.getPosition(), 
                frontRightModule.getPosition(), 
                backRightModule.getPosition()
            }); 



        // Setting default command
        this.setDefaultCommand(new DefaultDriveCommand(this));

        // Defing module state publisher 
        moduleStatesPublisher = NetworkTableInstance.getDefault().getStructArrayTopic(
            "Module States", SwerveModuleState.struct).publish();

        targetStatesPublisher = NetworkTableInstance.getDefault().getStructArrayTopic(
            "Target States", SwerveModuleState.struct).publish();

        robotPosePublisher = NetworkTableInstance.getDefault().getStructTopic(
            "Robot Pose", Pose2d.struct).publish(); 

        
    }

    /**
     * Sets the swerve modules to the state needed for driving 
     * 
     * @param vertical The component of speed away from alliance wall 
     * @param horizontal The component of speed to the left of the alliance wall
     * @param rotation The the angular velocity of the robot CCW+
     */
    public void drive(double vertical, double horizontal, double rotation) {
        // Generating the nessasery modules states for the given speeds
        SwerveModuleState[] targetStates = m_DriveKinematics.toSwerveModuleStates( 
            ChassisSpeeds.fromFieldRelativeSpeeds(vertical, horizontal, rotation, gyro.getRotation2d())
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
        backLeftModule.setState(targetStates[1]);
        frontRightModule.setState(targetStates[2]);        
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
        }); 

        // Updating the pose info in the odometery 
        m_DriveOdometry.update(gyro.getRotation2d(), 
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(), 
            backLeftModule.getPosition(), 
            frontRightModule.getPosition(), 
            backRightModule.getPosition()
        }); 

        // Publishing the current robot pose 
        robotPosePublisher.set(
            m_DriveOdometry.getPoseMeters()
        );

        // Publishing the Encoder readings 
        SmartDashboard.putNumber("Front Left Angle", frontLeftModule.getAbsoluteAngle().getDegrees());
        SmartDashboard.putNumber("Front Right Angle", frontRightModule.getAbsoluteAngle().getDegrees());
        SmartDashboard.putNumber("Back Left Angle", backLeftModule.getAbsoluteAngle().getDegrees());
        SmartDashboard.putNumber("Back Right Angle", backRightModule.getAbsoluteAngle().getDegrees());

        
       
 
    }

    /**
     * Calculates the module states needed to acheive the target speed 
     * (Resolved error with wpilib guess this is useless now)
     * 
     * @param targetSpeed The speed of the robot try to be acheived as a ChassisSpeeds
     * @return The swerve module states need to acheive the target speed
     */
    @Deprecated
    private SwerveModuleState[] getTargetStates(ChassisSpeeds targetSpeed) {
        double velocityRotation = (targetSpeed.omegaRadiansPerSecond * DriveConstants.robotRadius); 

        double[] moduleAngles = {
            (3 * Math.PI) / 4,
            (Math.PI) / 4,  
            (5 * Math.PI) / 4,
            (7 * Math.PI) / 4
        }; 

        double[][] moduleVelocitiesComponents = new double[4][2];
        double[] moduleVelocities = new double[4];  

        moduleVelocitiesComponents[0][0] = (targetSpeed.vxMetersPerSecond + velocityRotation * Math.sin(moduleAngles[0])); 
        moduleVelocitiesComponents[0][1] = (targetSpeed.vyMetersPerSecond + velocityRotation * Math.cos(moduleAngles[0])); 

        moduleVelocitiesComponents[1][0] = (targetSpeed.vxMetersPerSecond + velocityRotation * Math.cos(moduleAngles[1])); 
        moduleVelocitiesComponents[1][1] = (targetSpeed.vyMetersPerSecond + velocityRotation * Math.sin(moduleAngles[1])); 
        
        moduleVelocitiesComponents[2][0] = (targetSpeed.vxMetersPerSecond + velocityRotation * Math.cos(moduleAngles[2])); 
        moduleVelocitiesComponents[2][1] = (targetSpeed.vyMetersPerSecond + velocityRotation * Math.sin(moduleAngles[2]));

        moduleVelocitiesComponents[3][0] = (targetSpeed.vxMetersPerSecond + velocityRotation * Math.sin(moduleAngles[3])); 
        moduleVelocitiesComponents[3][1] = (targetSpeed.vyMetersPerSecond + velocityRotation * Math.cos(moduleAngles[3])); 

        for (int i = 0; i < 4; i++) {
            moduleAngles[i] = Math.atan(moduleVelocitiesComponents[i][1] / moduleVelocitiesComponents[i][0]);
            moduleVelocities[i] = Math.sqrt(
                moduleVelocitiesComponents[i][0] * moduleVelocitiesComponents[i][0] + 
                moduleVelocitiesComponents[i][1] * moduleVelocitiesComponents[i][1] 
            ); 

            if (moduleAngles[i] < 0) {
                moduleAngles[i] += (2 * Math.PI); 
            } 

            if (moduleVelocitiesComponents[i][0] < 0) {
                if (moduleVelocitiesComponents[i][1] < 0) {
                    moduleAngles[i] += (Math.PI); 
                }

                else {
                    moduleAngles[i] -= Math.PI; 

                }
            } else if (moduleVelocitiesComponents[i][1] < 0) {
                
            }

        }
        

        return new SwerveModuleState[] {
            new SwerveModuleState(moduleVelocities[0], new Rotation2d(moduleAngles[0])),
            new SwerveModuleState(moduleVelocities[2], new Rotation2d(moduleAngles[1])),
            new SwerveModuleState(moduleVelocities[1], new Rotation2d(moduleAngles[2])),
            new SwerveModuleState(moduleVelocities[3], new Rotation2d(moduleAngles[3])),
        };
    }
    
}
