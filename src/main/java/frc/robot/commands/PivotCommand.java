package frc.robot.commands;

import frc.robot.Input;

//Able to lower to intake, Spin the motors, Bring back the intake

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class PivotCommand extends Command {

    private final IntakeSubsystem m_IntakeSubsystem;
    private final Timer finishTimer; 

    public PivotCommand(IntakeSubsystem m_IntakeSubsystem) {
        this.m_IntakeSubsystem = m_IntakeSubsystem;
        finishTimer = new Timer(); 
        this.addRequirements(m_IntakeSubsystem); 
    }
    
    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        m_IntakeSubsystem.pivotintake(-0.15);
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
        return finishTimer.get() >= 0.5;
    }


}