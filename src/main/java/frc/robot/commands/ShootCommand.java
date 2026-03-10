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
      m_ShooterSubsystem.spinFlywheel(ShooterConstants.flywheelSpeed);

      if (m_ShooterSubsystem.getFlyWheelVelocity() <= -400) {
        m_ShooterSubsystem.spinIndexer(-0.3);
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
