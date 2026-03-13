package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Input;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class DefaultDriveCommand extends Command {
     private final DriveSubsystem m_DriveSubsystem; 

    public DefaultDriveCommand(DriveSubsystem m_DriveSubsystem) {
      this.m_DriveSubsystem = m_DriveSubsystem; 
      this.addRequirements(m_DriveSubsystem);
    }

    @Override 
    public void initialize() {
    }

    @Override 
    public void execute() {
        
      m_DriveSubsystem.drive(Input.getVertical(), Input.getHorizontal(), Input.getRotation());
  
    }

    @Override
    public void end(boolean interrupted) {
     
    }

    @Override 
    public boolean isFinished() {
      return false;
    }
    
}
