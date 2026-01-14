package frc.robot.subsystems.swerve;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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
    
    /**
     * Constructs a SwerveModule object. 
     * 
     * @param driveCANID The CAN ID of the drive motor.
     * @param angleMotorCANID The CAN ID of the angle motor.
     * @param encoderID The rio port for the absolute encoder. 
     * @param offset The absolute encoder offset. 
     */
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

    /**
     * Sets the Swerve Module to a given state. 
     * 
     * @param targetState The final state of the module.  
     */
    public void setState(SwerveModuleState targetState) {
        // Setting the drive and angle motors using PID controllers and the target module state
        driveMotor.set(drivePID.calculate(driveEncoder.getVelocity(), 
            (targetState.speedMetersPerSecond / DriveConstants.wheelRadius) * DriveConstants.rpmConversionFactor
        ));


        angleMotor.set(anglePID.calculate(getAbsoluteAngle().getDegrees(), targetState.angle.getDegrees()));

    }

    /**
     * Gets the angle reading from the absolute encoder
     * 
     * @return The angle reading from the absolute encoder as a Rotation2d. 
     */
    public Rotation2d getAbsoluteAngle() {
        return new Rotation2d(Units.degreesToRadians(absoluteEncoder.get() + encoderOffset));
    }

    /**
     * Gets the angle reading from the relative encoder
     * 
     * @return The angle reading from the relative encoder as a Rotation2d. 
     */
    public Rotation2d getRelativeAngle() {
        return new Rotation2d(Units.degreesToRadians(angleEncoder.getPosition())); 
    }

    /**
     * Gets the velocity of the wheel from the drive encoder
     * 
     * @return The velocity of the wheel in RPM
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity(); 
    }

    /**
     * Gets the current state of the module
     * 
     * @return The current module state as a SwerveModuleState
     */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
            (driveEncoder.getVelocity() / DriveConstants.rpmConversionFactor) * DriveConstants.wheelRadius, 
            getAbsoluteAngle() 
        );
    }

    /**
     * Gets the current position of the module
     * 
     * @return The current position as a SwerveModulePosition
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition() * (Math.PI * 2 * DriveConstants.wheelRadius),
            getAbsoluteAngle()
        ); 
    }

    
    
}
