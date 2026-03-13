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
    private final SwerveDrive swerveDrive; 

    public DriveSubsystem() {
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
        try {
            this.swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(DriveConstants.maxSpeed);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        
    }

    public void drive(double vertical, double horizontal, double rotation) {
        swerveDrive.driveFieldOriented(new ChassisSpeeds(vertical, horizontal, rotation));
    }
    
}
