package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DefaultDriveCommand;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DriveSubsystem extends SubsystemBase {

    private final File swerveDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    private final SwerveDrive swerveDrive; 

    public DriveSubsystem() throws IOException {
        swerveDrive = new SwerveParser(swerveDirectory).createSwerveDrive(DriveConstants.MAX_SPEED);
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        this.setDefaultCommand(new DefaultDriveCommand(this));
    }

    public void drive(double horizontal, double vertical, double rotation) {
       ChassisSpeeds targetSpeeds = new ChassisSpeeds(horizontal, vertical, rotation); 
       swerveDrive.driveFieldOriented(targetSpeeds);
    }

    
}
