package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Input;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.swerve.DriveSubsystem;


public class AlignHubCommand extends Command {
    private final PIDController headingController;

    private final DriveSubsystem m_DriveSubsystem;

    private double targetAngle;

    public AlignHubCommand(DriveSubsystem m_DriveSubsystem) {
        this.m_DriveSubsystem = m_DriveSubsystem;
        addRequirements(m_DriveSubsystem);

        headingController = new PIDController(
                DriveConstants.kHeadingP,
                DriveConstants.kHeadingI,
                DriveConstants.kHeadingD);
    }

    @Override 
    public void initialize() {
        Pose2d robotPose = m_DriveSubsystem.getCurrentPose(); 
        
        // Get the target angle for which ever hub is on our allience 
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {

                targetAngle = Math.abs(Math.atan2(
                    DriveConstants.hubPoseRed.getY() - robotPose.getY(), 
                    DriveConstants.hubPoseRed.getX() - robotPose.getX()
                )); 
            }
            if (ally.get() == Alliance.Blue) {

                targetAngle = Math.abs(Math.atan2(
                    DriveConstants.hubPoseBlue.getY() - robotPose.getY(), 
                    DriveConstants.hubPoseBlue.getX() - robotPose.getX()
                )); 
            }
        }
        else {
            this.cancel();
        }


    }

    @Override
    public void execute() {
        m_DriveSubsystem.drive(0, 0, 
            headingController.calculate(
                m_DriveSubsystem.getCurrentPose().getRotation().getDegrees(),
                targetAngle
            )
        );
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        if (targetAngle == m_DriveSubsystem.getCurrentPose().getRotation().getDegrees()) {
            return true; 
        }
        else {
            return false;
        }
        
    }

}
