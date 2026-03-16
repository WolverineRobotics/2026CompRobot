package frc.robot.commands;

import frc.robot.Input;

//Able to lower to intake, Spin the motors, Bring back the intake

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class PivotCommand extends Command {

    private final IntakeSubsystem m_IntakeSubsystem;

    public PivotCommand(IntakeSubsystem m_IntakeSubsystem) {
        this.m_IntakeSubsystem = m_IntakeSubsystem;
        this.addRequirements(m_IntakeSubsystem); 
    }
    
    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        m_IntakeSubsystem.pivotintake(Input.pivotIntake());
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return Input.pivotIntake() == 0;
    }


}