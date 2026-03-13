package frc.robot.subsystems;


import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem  extends SubsystemBase {

    private final SparkMax flywheelMotor; 
    private final SparkMax indexerMotor; 

    private final SparkMaxConfig indexerConfig; 

    private final RelativeEncoder flywheelEncoder; 

    private final PIDController flywheelController; 


    public ShooterSubsystem() {
        flywheelMotor = new SparkMax(
            ShooterConstants.flywheelCANID, 
            MotorType.kBrushless
        ); 

        indexerMotor = new SparkMax(
            ShooterConstants.indexerCANID, 
            MotorType.kBrushless
        ); 

        flywheelEncoder = flywheelMotor.getEncoder(); 

        flywheelController = new PIDController(
            ShooterConstants.flyWheelKp, 
            ShooterConstants.flyWheelKi,
            ShooterConstants.flyWheelKd
        ); 

        indexerConfig = new SparkMaxConfig(); 
        indexerConfig.smartCurrentLimit(ShooterConstants.indexerCurrentLimit, ShooterConstants.indexerCurrentLimit);
        indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 
    }

    public void setFlyWheelSpeed(double targetSpeed) {
        flywheelMotor.set(
            flywheelController.calculate(
                getFlyWheelVelocity(), 
                targetSpeed
            )
        ); 
    }

    public void spinIndexer(double speed) {
        indexerMotor.set(speed);
    }

    public void spinFlywheel(double speed) {
        flywheelMotor.set(speed); 
    }

    public double getFlyWheelVelocity() {
        return Units.rotationsPerMinuteToRadiansPerSecond(flywheelEncoder.getVelocity()); 
    }

    @Override 
    public void periodic() {
        SmartDashboard.putNumber("Flywheel Velocity radps", getFlyWheelVelocity());
    }


}
