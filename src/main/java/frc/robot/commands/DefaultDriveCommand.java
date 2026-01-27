package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Input;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends Command {
    
    DriveSubsystem m_DriveSubsystem;


    public DefaultDriveCommand(DriveSubsystem driveBase) {
        this.m_DriveSubsystem = driveBase;
        addRequirements(driveBase);
        
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
        //driveBase.drive(0, 0, 0);
    }

    @Override 
    public boolean isFinished() {
        return false;
    }

}
