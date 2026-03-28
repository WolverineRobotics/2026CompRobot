package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Input;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class OuttakeCommand extends Command {

    private final IntakeSubsystem m_IntakeSubsystem;

    public OuttakeCommand(IntakeSubsystem m_IntakeSubsystem) {
        this.m_IntakeSubsystem = m_IntakeSubsystem; 
        this.addRequirements(m_IntakeSubsystem);
    }
    
    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        m_IntakeSubsystem.spinRoller(-(IntakeConstants.intakeSpeed * 2));
    }

    @Override
    public void end(boolean interrupted){
        m_IntakeSubsystem.spinRoller(0);
    }

    @Override
    public boolean isFinished(){
        return !Input.startOuttaking(); 
    }


}