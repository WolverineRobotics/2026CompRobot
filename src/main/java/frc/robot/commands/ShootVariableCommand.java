package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Input;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class ShootVariableCommand extends Command {

    private final ShooterSubsystem m_ShooterSubsystem; 
    private final DriveSubsystem m_DriveSubsystem;
    private double distance; 
    private double targetAngularVelocity; 
    private final StructPublisher<Pose2d> hubPosePublisher; 
    private Pose2d hubPose;

    public ShootVariableCommand(ShooterSubsystem m_ShooterSubsystem, DriveSubsystem m_DriveSubsystem) {
      this.m_ShooterSubsystem = m_ShooterSubsystem; 
      this.m_DriveSubsystem = m_DriveSubsystem; 
      distance = 0; 
      targetAngularVelocity = 0; 
      hubPose = new Pose2d();

      hubPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Hub Pose", Pose2d.struct).publish();
    }

    @Override 
    public void initialize() {
      Optional<Alliance> ally = DriverStation.getAlliance();
      targetAngularVelocity = m_ShooterSubsystem.getTargetVelocity(
          m_DriveSubsystem.getHubDistance(ally.get() == Alliance.Red)
      );  

      

    }

    @Override 
    public void execute() {
        

     m_ShooterSubsystem.setFlyWheelSpeed(targetAngularVelocity);
     if (m_ShooterSubsystem.getFlyWheelVelocity() > Math.abs(targetAngularVelocity)) {
         m_ShooterSubsystem.spinIndexer(ShooterConstants.defaultIndexerSpeed);
     }
   
    SmartDashboard.putNumber("Target Angular Velocity", targetAngularVelocity);
    SmartDashboard.putNumber("Distance", distance);
    hubPosePublisher.set(hubPose);
 
      
  
    }

    @Override
    public void end(boolean interrupted) {
      m_ShooterSubsystem.spinFlywheel(0);
      m_ShooterSubsystem.spinIndexer(0);
    }

    @Override 
    public boolean isFinished() {
        return !Input.spinFlywheel();
    }
    
}
