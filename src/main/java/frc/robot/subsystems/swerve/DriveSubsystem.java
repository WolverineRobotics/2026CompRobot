package frc.robot.subsystems.swerve;



import java.util.Vector;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.KalmanFilter;
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
        
        setupPathplanner();



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
        
        // Setting each swerve module to the correct state
        frontLeftModule.setState(targetStates[0]);
        backLeftModule.setState(targetStates[1]);
        frontRightModule.setState(targetStates[2]);        
        backRightModule.setState(targetStates[3]);
    }

    private void driveRobotOriented(ChassisSpeeds speeds) {
        SwerveModuleState[] targetStates = m_DriveKinematics.toSwerveModuleStates(speeds); 

        frontLeftModule.setState(targetStates[0]);
        backLeftModule.setState(targetStates[1]);
        frontRightModule.setState(targetStates[2]);
        backRightModule.setState(targetStates[3]);
    }

    /**
     * Method to get the pose of the robot form the limelight
     * 
     * @return Pose of the robot as Pose3d 
     */
    public PoseEstimate getVisionPoseEstimate() {
        LimelightHelpers.SetRobotOrientation(getName(), gyro.getYaw().getValueAsDouble(), 0,
        gyro.getPitch().getValueAsDouble(), 0,
        gyro.getRoll().getValueAsDouble(), 0);

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

    public void setPose(Pose2d updatedPose) {
        m_DriveOdometry.resetPose(updatedPose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return m_DriveKinematics.toChassisSpeeds(
            new SwerveModuleState[] {
                frontLeftModule.getModuleState(), 
                backLeftModule.getModuleState(),
                frontRightModule.getModuleState(), 
                backRightModule.getModuleState()
            }
        ); 
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

        
       

       SmartDashboard.putNumber("Current Angle", frontLeftModule.getAbsoluteAngle().getDegrees()); 
       SmartDashboard.putNumber("Current Speed", frontLeftModule.getDriveVelocity()); 
    }

   private void setupPathplanner() {
        RobotConfig config; 
        try {
            config = RobotConfig.fromGUISettings(); 
        } catch (Exception e) {
            e.printStackTrace();
        }
        AutoBuilder.configure(
            this::getOdometeryPose,
            this::setPose,
            this::getRobotRelativeSpeeds, 
            (speeds, feedforwards) -> driveRobotOriented(speeds), 
            new PPHolonomicDriveController(
                new PIDConstants(5, 0, 0), 
                new PIDConstants(0, 0, 0)
            ),
            config, 
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
            this
        ); 
   }
}
