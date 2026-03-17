package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Input;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class ShootCommand extends Command {

    private final ShooterSubsystem m_ShooterSubsystem; 
    private final DriveSubsystem m_DriveSubsystem;
    private double distance; 
    private double targetAngularVelocity; 

    public ShootCommand(ShooterSubsystem m_ShooterSubsystem, DriveSubsystem m_DriveSubsystem) {
      this.m_ShooterSubsystem = m_ShooterSubsystem; 
      this.m_DriveSubsystem = m_DriveSubsystem; 
      distance = 0; 
      targetAngularVelocity = 0; 
    }

    @Override 
    public void initialize() {
      Pose2d hubPose = new Pose2d(); 
      Pose2d botPose = m_DriveSubsystem.getBotPose(); 

      Optional<Alliance> ally = DriverStation.getAlliance();
      if (ally.isPresent()) {
          if (ally.get() == Alliance.Red) {
             hubPose = DriveConstants.hubPoseRed;
          }
          if (ally.get() == Alliance.Blue) {
             hubPose = DriveConstants.hubPoseBlue;
          }
      }
      else {
          hubPose = DriveConstants.hubPoseBlue; 
      }

      distance = (
        (hubPose.getY() - botPose.getY()) * (hubPose.getY() - botPose.getY()) +
        (hubPose.getX() - botPose.getX()) * (hubPose.getX() - botPose.getX())
      ); 

      targetAngularVelocity = m_ShooterSubsystem.getTargetVelocity(distance); 

      

    }

    @Override 
    public void execute() {
        

     m_ShooterSubsystem.setFlyWheelSpeed(targetAngularVelocity);
     if (m_ShooterSubsystem.getFlyWheelVelocity() > Math.abs(targetAngularVelocity)) {
         m_ShooterSubsystem.spinIndexer(ShooterConstants.defaultIndexerSpeed);
     }
   
    SmartDashboard.putNumber("Target Angular Velocity", targetAngularVelocity);
    SmartDashboard.putNumber("Distance", distance);
 
      
  
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
