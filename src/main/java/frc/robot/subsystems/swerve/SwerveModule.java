package frc.robot.subsystems.swerve;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;

public class SwerveModule {

    // Declaring the Angle and Drive Motors
    private final SparkMax driveMotor; 
    private final SparkMax angleMotor; 

    // Declaring the Encoders 
    private final AnalogEncoder absoluteEncoder; 
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder angleEncoder;

    // Declaring the PID controllers
    private final PIDController drivePID; 
    private final PIDController anglePID; 

    // Declaring Encoder Offsets
    private final double encoderOffset; 
    

    public SwerveModule(int driveCANID, int angleMotorCANID, int encoderID, double offset) {
        // Defining motor with their CAN IDs
        driveMotor = new SparkMax(driveCANID, MotorType.kBrushless); 
        angleMotor = new SparkMax(angleMotorCANID, MotorType.kBrushless); 

        // Defining the absolute encoder with the encoder id
        absoluteEncoder = new AnalogEncoder(encoderID);

        // Defining relative encoders from motors         
        driveEncoder = driveMotor.getEncoder(); 
        angleEncoder = angleMotor.getEncoder(); 

        // Defining PID controllers with values from constants
        drivePID = new PIDController(
            DriveConstants.kDriveP,
            DriveConstants.kDriveI,
            DriveConstants.kDriveD
        ); 

        anglePID = new PIDController(
            DriveConstants.kAngleP,
            DriveConstants.kAngleI,
            DriveConstants.kAngleD
        ); 

        // Defining encoder offset
        encoderOffset = offset; 

    }

    
    public void setState(SwerveModuleState targetState) {
        // Setting the drive and angle motors using PID controllers and the target module state
        driveMotor.set(drivePID.calculate(driveEncoder.getVelocity(), 
            (targetState.speedMetersPerSecond / DriveConstants.wheelRadius) * DriveConstants.rpmConversionFactor
        ));


        angleMotor.set(anglePID.calculate(getAbsoluteAngle().getDegrees(), targetState.angle.getDegrees()));

    }

    // Getters for angle with both absolute and relative
    public Rotation2d getAbsoluteAngle() {
        return new Rotation2d(Units.degreesToRadians(absoluteEncoder.get() + encoderOffset));
    }

    public Rotation2d getRelativeAngle() {
        return new Rotation2d(Units.degreesToRadians(angleEncoder.getPosition())); 
    }

    // Getter for velocity of drive wheel
    public double getDriveVelocity() {
        return driveEncoder.getVelocity(); 
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
            (driveEncoder.getVelocity() / DriveConstants.rpmConversionFactor) * DriveConstants.wheelRadius, 
            getAbsoluteAngle() 
        );
    }

    
    
}
