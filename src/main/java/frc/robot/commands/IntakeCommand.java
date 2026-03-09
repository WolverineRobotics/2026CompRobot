package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Input;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

    private final IntakeSubsystem m_IntakeSubsystem;

    public IntakeCommand(IntakeSubsystem m_IntakeSubsystem) {
        this.m_IntakeSubsystem = m_IntakeSubsystem; 
    }
    
    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        m_IntakeSubsystem.intakeGamepiece();;
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return Input.stopIntaking(); 
    }


}