package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveTimedCommand extends Command {

    private final DriveSubsystem m_DriveSubsystem; 
    private final Timer finishTimer; 
    private final double[] velocity; 
    private final double time; 
    private final double angularVelocity; 

    public DriveTimedCommand(DriveSubsystem m_DriveSubsystem, double[] velocity, double time, double angularVelocity) {
        this.m_DriveSubsystem = m_DriveSubsystem; 
        finishTimer = new Timer(); 
        this.velocity = velocity; 
        this.time = time; 
        this.angularVelocity = angularVelocity;
        addRequirements(m_DriveSubsystem);
    }

     @Override 
    public void initialize() {
        finishTimer.start();
    }

    @Override 
    public void execute() {
        m_DriveSubsystem.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                velocity[0], 
                velocity[1], 
                angularVelocity, 
                m_DriveSubsystem.getBotPose().getRotation()
            )
        );
  
    }

    @Override
    public void end(boolean interrupted) {
        m_DriveSubsystem.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                0, 
                0, 
                0, 
                m_DriveSubsystem.getBotPose().getRotation()
            )
        );
      
    }

    @Override 
    public boolean isFinished() {
        return finishTimer.get() >= time; 
    }
    
}
