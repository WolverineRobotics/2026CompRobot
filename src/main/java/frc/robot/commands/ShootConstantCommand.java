package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Input;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class ShootConstantCommand extends Command {

    private final ShooterSubsystem m_ShooterSubsystem; 
    private final double angularVelocity; 
    private final Timer finishTimer; 
   
  

    public ShootConstantCommand(ShooterSubsystem m_ShooterSubsystem, double angularVelocity) {
      this.m_ShooterSubsystem = m_ShooterSubsystem; 
      this.angularVelocity = angularVelocity; 
      finishTimer = new Timer(); 
     
    
    }

    @Override 
    public void initialize() {
          

    }

    @Override 
    public void execute() {
        

     m_ShooterSubsystem.setFlyWheelSpeed(angularVelocity);
     if (m_ShooterSubsystem.getFlyWheelVelocity() > Math.abs(angularVelocity)) {

         m_ShooterSubsystem.spinIndexer(ShooterConstants.defaultIndexerSpeed);
         
     
  
    }
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
