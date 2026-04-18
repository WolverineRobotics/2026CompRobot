package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Input;
import frc.robot.subsystems.IntakeSubsystem;

public class PivotManualCommand extends Command {

    private final IntakeSubsystem m_IntakeSubsystem;


    public PivotManualCommand(IntakeSubsystem m_IntakeSubsystem) {
        this.m_IntakeSubsystem = m_IntakeSubsystem;
        this.addRequirements(m_IntakeSubsystem); 
    }
    
    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        m_IntakeSubsystem.pivotintake(-Input.getPivotSpeed());

    }

    @Override
    public void end(boolean interrupted){
        m_IntakeSubsystem.pivotintake(0);
    }

    @Override
    public boolean isFinished(){
        return Input.getPivotSpeed() == 0;
    }


}
