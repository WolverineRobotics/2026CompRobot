package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Input;
import frc.robot.subsystems.IntakeSubsystem;

public class PivotManualCommand extends Command {

    private final IntakeSubsystem m_IntakeSubsystem;
    private final Timer finishTimer; 


    public PivotManualCommand(IntakeSubsystem m_IntakeSubsystem) {
        this.m_IntakeSubsystem = m_IntakeSubsystem;
        finishTimer = new Timer(); 
        this.addRequirements(m_IntakeSubsystem); 
    }
    
    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        m_IntakeSubsystem.pivotintake(-Input.getPivotSpeed());
        if (m_IntakeSubsystem.atBumpers()) {
            if (finishTimer.get() == 0) {
                finishTimer.start();
            }
        }

        else {
            finishTimer.reset();
        }

    }

    @Override
    public void end(boolean interrupted){
        m_IntakeSubsystem.pivotintake(0);
    }

    @Override
    public boolean isFinished(){
        return Input.getPivotSpeed() == 0 || finishTimer.get() >= 0.5;
    }


}
