package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Input;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class DefaultDriveCommand extends Command {

    DriveSubsystem m_DriveSubsystem; 

    public DefaultDriveCommand(DriveSubsystem m_DriveSubsystem) {
        this.m_DriveSubsystem = m_DriveSubsystem; 
        addRequirements(m_DriveSubsystem);
    }

     @Override 
    public void initialize() {
    }

    @Override 
    public void execute() {
        m_DriveSubsystem.drive(
             ChassisSpeeds.fromFieldRelativeSpeeds(
                                Input.getVertical() * DriveConstants.maxSpeed, 
                                Input.getHorizontal() * DriveConstants.maxSpeed, 
                                Input.getRotation() * DriveConstants.maxAngularVelocity, 
                                m_DriveSubsystem.getGyroReading()
                        )
        ); 
  
    }

    @Override
    public void end(boolean interrupted) {
      
    }

    @Override 
    public boolean isFinished() {
        return false;
    }
    
}
