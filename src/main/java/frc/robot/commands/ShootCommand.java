package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Input;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {

    private final ShooterSubsystem m_ShooterSubsystem; 


    public ShootCommand(ShooterSubsystem m_ShooterSubsystem) {
      this.m_ShooterSubsystem = m_ShooterSubsystem; 
    }

    @Override 
    public void initialize() {
    }

    @Override 
    public void execute() {
      m_ShooterSubsystem.setFlyWheelSpeed(Input.spinFlywheel());

      if (m_ShooterSubsystem.getFlyWheelVelocity() == Input.spinFlywheel()) {
        m_ShooterSubsystem.spinIndexer(ShooterConstants.defaultIndexerSpeed);
      }
      // IntakeSubsystem.pivotintake(); 
  
    }

    @Override
    public void end(boolean interrupted) {
      
    }

    @Override 
    public boolean isFinished() {
        return false;
    }
    
}
