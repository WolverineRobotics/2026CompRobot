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
import edu.wpi.first.math.estimator.PoseEstimator;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.commands.drive.DefaultDriveCommand;

public class DriveSubsystem extends SubsystemBase {

        private final SwerveModule frontLeftModule; 
        private final SwerveModule frontRightModule; 
        private final SwerveModule backLeftModule; 
        private final SwerveModule backRightModule; 

        private final SwerveDriveKinematics kinematics; 
        private final SwerveDriveOdometry odometry;
        private final SwerveDrivePoseEstimator poseEstimator; 


        private final Pigeon2 gyro; 

        private final StructArrayPublisher<SwerveModuleState> modulePublisher; 
        private final StructArrayPublisher<SwerveModuleState> targetPublisher; 
        private final StructPublisher<Pose2d> posePublisher; 
        private final StructPublisher<Pose2d> purePublisher; 

        private double lastYaw; 
        private double lastPitch; 
        private double lastRoll; 

     

        public DriveSubsystem() {
                frontLeftModule = new SwerveModule(
                        DriveConstants.frontLeftDriveID, 
                        DriveConstants.frontLeftAngleID,
                        DriveConstants.frontLeftAbsoluteEncoder, 
                        DriveConstants.frontLeftEncoderOffset, 
                        DriveConstants.frontLeftInverted,
                        DriveConstants.frontLeftDriveInverted
                );
                frontRightModule = new SwerveModule(
                        DriveConstants.frontRightDriveID, 
                        DriveConstants.frontRightAngleID,
                        DriveConstants.frontRightAbsoluteEncoder, 
                        DriveConstants.frontRightEncoderOffset, 
                        DriveConstants.frontRightInverted,
                        DriveConstants.frontRightDriveInverted
                );
                backLeftModule = new SwerveModule(
                        DriveConstants.backLeftDriveID, 
                        DriveConstants.backLeftAngleID,
                        DriveConstants.backLeftAbsoluteEncoder, 
                        DriveConstants.backLeftEncoderOffset, 
                        DriveConstants.backLeftInverted,
                        DriveConstants.backLeftDriveInverted
                );
                backRightModule = new SwerveModule(
                        DriveConstants.backRightDriveID, 
                        DriveConstants.backRightAngleID,
                        DriveConstants.backRightAbsoluteEncoder, 
                        DriveConstants.backRightEncoderOffset, 
                        DriveConstants.backRightInverted,
                        DriveConstants.backRightDriveInverted
                );

                kinematics = new SwerveDriveKinematics(
                        new Translation2d(DriveConstants.xTranslation, DriveConstants.yTranslation),
                        new Translation2d(DriveConstants.xTranslation, -DriveConstants.yTranslation),
                        new Translation2d(-DriveConstants.xTranslation, DriveConstants.yTranslation),
                        new Translation2d(-DriveConstants.xTranslation, -DriveConstants.yTranslation)

                );

                gyro = new Pigeon2(DriveConstants.gyroID); 

                odometry = new SwerveDriveOdometry(
                        kinematics, 
                        gyro.getRotation2d(), 
                        new SwerveModulePosition[] {
                                frontLeftModule.getPosition(),
                                frontRightModule.getPosition(),
                                backLeftModule.getPosition(),
                                backRightModule.getPosition(),
                        }
                );

                this.setDefaultCommand(new DefaultDriveCommand(this));

                RobotConfig config; 
                try{
                        config = RobotConfig.fromGUISettings();
                        AutoBuilder.configure(
                        this::getBotPose, 
                        this::resetPose,
                        this::getRobotRelativeSpeeds,
                        (speeds, feedforwards) -> driveRobotOriented(speeds),
                        new PPHolonomicDriveController(
                                new PIDConstants(5, 0, 0), 
                                new PIDConstants(5, 0, 0)
                        ),
                        config,
                        () -> {
                                var alliance = DriverStation.getAlliance();
                                if (alliance.isPresent()) {
                                        return alliance.get() == DriverStation.Alliance.Red;
                                }
                                return false;
                        },
                        this
                );
                } catch (Exception e) {
                        e.printStackTrace();
                }

                

                

                modulePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
                targetPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/TargetStates", SwerveModuleState.struct).publish();
                posePublisher = NetworkTableInstance.getDefault().getStructTopic("Bot Pose", Pose2d.struct).publish(); 
                purePublisher = NetworkTableInstance.getDefault().getStructTopic("Lime Pose", Pose2d.struct).publish(); 

                lastYaw = 0; 
                lastRoll = 0; 
                lastPitch = 0; 

                poseEstimator = new SwerveDrivePoseEstimator(
                        kinematics,
                        gyro.getRotation2d(),
                        new SwerveModulePosition[] {
                                frontLeftModule.getPosition(),
                                frontRightModule.getPosition(),
                                backLeftModule.getPosition(),
                                backRightModule.getPosition(),
                        }, 
                        odometry.getPoseMeters()
                );

                
        
        }
        public void drive(ChassisSpeeds targetSpeeds) {
                SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(
                       targetSpeeds
                ); 

                

                targetPublisher.set(targetStates);

                
                SmartDashboard.putNumber("Front Left Target Drive", Units.radiansPerSecondToRotationsPerMinute(targetStates[0].speedMetersPerSecond * DriveConstants.wheelRadius));
                frontLeftModule.setState(targetStates[0]);
                frontRightModule.setState(targetStates[1]);
                backLeftModule.setState(targetStates[2]); 
                backRightModule.setState(targetStates[3]);


        }
        public void PIDDebugger() {
                frontLeftModule.setState(new SwerveModuleState(0, new Rotation2d(90)));
        }

        private void driveRobotOriented(ChassisSpeeds targetSpeeds) {
                SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds); 

                targetPublisher.set(targetStates);

                
                SmartDashboard.putNumber("Front Left Target Drive", (targetStates[0].speedMetersPerSecond * DriveConstants.wheelRadius) * DriveConstants.rpmConversionFactor);
                frontLeftModule.setState(targetStates[0]);
                frontRightModule.setState(targetStates[1]);
                backLeftModule.setState(targetStates[2]);
                backRightModule.setState(targetStates[3]);


        }

        public ChassisSpeeds getRobotRelativeSpeeds() {
                return kinematics.toChassisSpeeds(
                        new SwerveModuleState[] {
                                frontLeftModule.getModuleState(),
                                frontRightModule.getModuleState(),
                                backLeftModule.getModuleState(),
                                backRightModule.getModuleState(),
                        }
                );
        }

        public void resetPose(Pose2d targetPose) {
                odometry.resetPosition(
                        gyro.getRotation2d(),
                        new SwerveModulePosition[] {
                                frontLeftModule.getPosition(),
                                frontRightModule.getPosition(),
                                backLeftModule.getPosition(),
                                backRightModule.getPosition(),
                        } , targetPose);;
        }
        
        public Pose2d getBotPose() {
         return poseEstimator.getEstimatedPosition(); 
        }

        @Override 
        public void periodic() {
                
                
                SmartDashboard.putNumber("Robot Angle", gyro.getYaw().getValueAsDouble());

                modulePublisher.set(
                        new SwerveModuleState[] {
                                frontLeftModule.getModuleState(),
                                frontRightModule.getModuleState(),
                                backLeftModule.getModuleState(),
                                backRightModule.getModuleState(),
                                
                        }
                );

                odometry.update(
                        gyro.getRotation2d(),
                        new SwerveModulePosition[] {
                            frontLeftModule.getPosition(),
                            frontRightModule.getPosition(),
                            backLeftModule.getPosition(),
                            backRightModule.getPosition(),    
                        }
                );

                SmartDashboard.putNumber("FL Drive", frontLeftModule.getDriveVelocity()); 
                SmartDashboard.putNumber("Robot Angle", gyro.getRotation2d().getDegrees());

                SmartDashboard.putNumber("FR Angle", frontRightModule.getAbsoluteAngle().getDegrees()); 
                SmartDashboard.putNumber("FL Angle", frontLeftModule.getAbsoluteAngle().getDegrees()); 
                SmartDashboard.putNumber("BR Angle", backRightModule.getAbsoluteAngle().getDegrees()); 
                SmartDashboard.putNumber("BL Angle", backLeftModule.getAbsoluteAngle().getDegrees()); 
                
                SmartDashboard.putBoolean("In Shooting Range", inShootingRange());
                SmartDashboard.putNumber("Distance", getHubDistance(true)); 

                double yawRate = (lastYaw - gyro.getYaw().getValueAsDouble()) / 0.2;
                double pitchRate = (lastPitch - gyro.getPitch().getValueAsDouble()) / 0.2;
                double rollRate = (lastRoll - gyro.getRoll().getValueAsDouble()) / 0.2;
        
                LimelightHelpers.SetRobotOrientation("limelight", 
                        gyro.getYaw().getValueAsDouble(), 
                        yawRate, 
                        gyro.getPitch().getValueAsDouble(), 
                        pitchRate,
                        gyro.getRoll().getValueAsDouble(), 
                        rollRate
                );
                
                purePublisher.set(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").pose);

                poseEstimator.addVisionMeasurement(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").pose, Timer.getFPGATimestamp());

                poseEstimator.update(
                        gyro.getRotation2d(), 
                        new SwerveModulePosition[] {
                                frontLeftModule.getPosition(),
                                frontRightModule.getPosition(), 
                                backLeftModule.getPosition(), 
                                backRightModule.getPosition()
                        }
                ); 

                posePublisher.set(getBotPose());

                lastYaw = gyro.getYaw().getValueAsDouble(); 
                lastPitch = gyro.getPitch().getValueAsDouble(); 
                lastRoll = gyro.getRoll().getValueAsDouble(); 

                

        
        }

        public double getHubDistance(boolean red) {
                if (red) {
                        return Math.hypot(
                        (DriveConstants.hubPoseRed.getY() - getBotPose().getY()),
                        (DriveConstants.hubPoseRed.getX() - getBotPose().getX())
                        ); 
                }

                else {
                        return Math.hypot(
                        (DriveConstants.hubPoseBlue.getY() - getBotPose().getY()),
                        (DriveConstants.hubPoseBlue.getX() - getBotPose().getX())
                        ); 
                }
        }

        public boolean inShootingRange() {
                var alliance = DriverStation.getAlliance(); 
                boolean red = false; 

                if (alliance.get() == Alliance.Red) {
                        red = true; 
                }

                if (DriveConstants.minShootingDistance <= getHubDistance(red) && 
                    getHubDistance(red) <= DriveConstants.maxShootingDistance) {
                        return true; 
                }

                else {
                        return false; 
                }

        }

}

