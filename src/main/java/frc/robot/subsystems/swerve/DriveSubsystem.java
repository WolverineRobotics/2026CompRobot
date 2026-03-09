package frc.robot.subsystems.swerve;

import java.util.Vector;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
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
    private final AHRS gyro; 
    // Declaring Pose Estimator
    private final SwerveDrivePoseEstimator poseEstimator;

    // Declaring publisher for module states
    private final StructArrayPublisher<SwerveModuleState> moduleStatesPublisher;
    private final StructArrayPublisher<SwerveModuleState> targetStatesPublisher;

    // Declaring publisher for Robot Pose
    private final StructPublisher<Pose2d> robotPosePublisher;

    private final StructPublisher<Rotation2d> robotHeadingPublisher; 

    /**
     * Constructs the Drive subsystem
     */
    public DriveSubsystem() {

        // Defining Swerve Module objects
        frontLeftModule = new SwerveModule(
            DriveConstants.frontLeftDriveID, 
            DriveConstants.frontLeftAngleID,
            DriveConstants.frontLeftAbsoluteEncoder, 
            DriveConstants.frontLeftEncoderOffset,
            DriveConstants.frontLeftInverted
        );

        frontRightModule = new SwerveModule(
            DriveConstants.frontRightDriveID, 
            DriveConstants.frontRightAngleID,
            DriveConstants.frontRightAbsoluteEncoder, 
            DriveConstants.frontRightEncoderOffset,
            DriveConstants.frontRightInverted
        );

        backLeftModule = new SwerveModule(
            DriveConstants.backLeftDriveID, 
            DriveConstants.backLeftAngleID,
            DriveConstants.backLeftAbsoluteEncoder, 
            DriveConstants.backLeftEncoderOffset,
            DriveConstants.backLeftInverted
        );

        backRightModule = new SwerveModule(
            DriveConstants.backRightDriveID, 
            DriveConstants.backRightAngleID,
            DriveConstants.backRightAbsoluteEncoder,
            DriveConstants.backRightEncoderOffset,
            DriveConstants.backRightInverted
        );

        // Defining Gyroscope
        gyro = new AHRS(NavXComType.kMXP_SPI);

        // Defining Swerve Kinematics and Odometery
        m_DriveKinematics = new SwerveDriveKinematics(
                new Translation2d(DriveConstants.xTranslation, DriveConstants.yTranslation), // Front Left
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

        RobotConfig config;
        try{
                config = RobotConfig.fromGUISettings();
                AutoBuilder.configure(
                this::getCurrentPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotOriented(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this // Reference to this subsystem to set requirements
        );
        } catch (Exception e) {
        // Handle exception as needed
                e.printStackTrace();
        }

        // Configure AutoBuilder last
        

        poseEstimator = new SwerveDrivePoseEstimator(
                m_DriveKinematics,
                getYaw(),
                new SwerveModulePosition[] {
                        frontLeftModule.getPosition(),
                        backLeftModule.getPosition(),
                        frontRightModule.getPosition(),
                        backRightModule.getPosition()
                },
                new Pose2d() // Initial Pose
        );

        // Setting default command
        this.setDefaultCommand(new DefaultDriveCommand(this));

        // Defing module state publisher
        moduleStatesPublisher = NetworkTableInstance.getDefault().getStructArrayTopic(
                "Module States", SwerveModuleState.struct).publish();

        robotHeadingPublisher = NetworkTableInstance.getDefault().getStructTopic("Robot Heading", Rotation2d.struct).publish();

        targetStatesPublisher = NetworkTableInstance.getDefault().getStructArrayTopic(
                "Target States", SwerveModuleState.struct).publish();

        robotPosePublisher = NetworkTableInstance.getDefault().getStructTopic(
                "Robot Pose", Pose2d.struct).publish();
        


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
        m_DriveOdometry.update(getYaw(),
                new SwerveModulePosition[] {
                        frontLeftModule.getPosition(),
                        backLeftModule.getPosition(),
                        frontRightModule.getPosition(),
                        backRightModule.getPosition()
                });

        // Publishing the current robot pose
        robotPosePublisher.set(
                poseEstimator.getEstimatedPosition());
        
        robotHeadingPublisher.set(gyro.getRotation2d());

        poseEstimator.update(getYaw(),
                new SwerveModulePosition[] {
                        frontLeftModule.getPosition(),
                        backLeftModule.getPosition(),
                        frontRightModule.getPosition(),
                        backRightModule.getPosition()
                });

        poseEstimator.addVisionMeasurement(getVisionPoseEstimate().pose, getVisionPoseEstimate().timestampSeconds);

        SmartDashboard.putNumber(" FL Current ANgle", frontLeftModule.getAbsoluteAngle().getDegrees());
        SmartDashboard.putNumber(" FR Current ANgle", frontRightModule.getAbsoluteAngle().getDegrees());
        SmartDashboard.putNumber(" BL Current ANgke", backLeftModule.getAbsoluteAngle().getDegrees());
        SmartDashboard.putNumber(" BR Current Angle", backRightModule.getAbsoluteAngle().getDegrees());
    }

    /**
     * Sets the swerve modules to the state needed for driving
     * 
     * @param vertical   The component of speed away from alliance wall
     * @param horizontal The component of speed to the left of the alliance wall
     * @param rotation The the angular velocity of the robot CCW+
     */
    public void drive(double vertical, double horizontal, double rotation) {
        // Generating the nessasery modules states for the given speeds
        SwerveModuleState[] targetStates = m_DriveKinematics.toSwerveModuleStates( 
            ChassisSpeeds.fromFieldRelativeSpeeds(
                vertical * DriveConstants.maxSpeed,
                horizontal * DriveConstants.maxSpeed,
                rotation * DriveConstants.maxAngularVelocity,
                gyro.getRotation2d()
            )
        ); 
      
        SmartDashboard.putNumber("FL Target Angle", targetStates[0].angle.getDegrees()); 
        SmartDashboard.putNumber("BL Target Angle", targetStates[1].angle.getDegrees()); 
        SmartDashboard.putNumber("FR Target Angle", targetStates[2].angle.getDegrees()); 
        SmartDashboard.putNumber("BR Target Angle", targetStates[3].angle.getDegrees()); 
        
        // Setting each swerve module to the correct state
        frontLeftModule.setState(targetStates[0]);
        backLeftModule.setState(targetStates[2]);
        frontRightModule.setState(targetStates[1]);
        backRightModule.setState(targetStates[3]);
    }

    private void driveRobotOriented(ChassisSpeeds speeds) {
        SwerveModuleState[] targetStates = m_DriveKinematics.toSwerveModuleStates(speeds);

        frontLeftModule.setState(targetStates[0]);
        backLeftModule.setState(targetStates[1]);
        frontRightModule.setState(targetStates[2]);
        backRightModule.setState(targetStates[3]);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return m_DriveKinematics.toChassisSpeeds(
                new SwerveModuleState[] {
                        frontLeftModule.getModuleState(),
                        backLeftModule.getModuleState(),
                        frontRightModule.getModuleState(),
                        backRightModule.getModuleState()
                });
    }



    public void resetPose(Pose2d updatedPose) {
        m_DriveOdometry.resetPose(updatedPose);
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition(); 
    }

    public Rotation2d getYaw() {
        return new Rotation2d(gyro.getAngle());
    }
    
    public void PIDDebugger() {
        SwerveModuleState targetState = new SwerveModuleState(0, new Rotation2d()); 
        SmartDashboard.putNumber("FL Target Angle", targetState.angle.getDegrees());
        frontLeftModule.setState(targetState);
    }

    /**
     * Method to get the pose estimate of the robot form the limelight
     * 
     * @return Pose estimate of the robot
     */
    public PoseEstimate getVisionPoseEstimate() {
        LimelightHelpers.SetRobotOrientation(getName(), gyro.getYaw(), 0,
                gyro.getPitch(), 0,
                gyro.getRoll(), 0);
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(getName());
    }

    /**
     * Method to get the pose of the robot from the encoders + gyro
     * 
     * @return Pose of the robot as Pose2d
     */
    public Pose2d getOdometeryPose() {
        return m_DriveOdometry.getPoseMeters();
    }


}
