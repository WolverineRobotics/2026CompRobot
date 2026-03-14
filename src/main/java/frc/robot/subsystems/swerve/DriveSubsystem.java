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

        private final SwerveModule frontLeftModule; 
        private final SwerveModule frontRightModule; 
        private final SwerveModule backLeftModule; 
        private final SwerveModule backRightModule; 

        private final SwerveDriveKinematics kinematics; 

     

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

                this.setDefaultCommand(new DefaultDriveCommand(this));
        
        }
        public void drive(double vertical, double horizontal, double rotation) {
                SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(
                        new ChassisSpeeds(
                                vertical * DriveConstants.maxSpeed, 
                                horizontal * DriveConstants.maxSpeed,
                                rotation * DriveConstants.maxAngularVelocity
                        )
                ); 

                

                frontLeftModule.setState(targetStates[0]);
                frontRightModule.setState(targetStates[1]);
                backLeftModule.setState(targetStates[2]);
                backRightModule.setState(targetStates[3]);


        }

}

