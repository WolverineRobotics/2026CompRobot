package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem  extends SubsystemBase {

    private final SparkMax flywheelMotor; 
    private final SparkMax indexerMotor; 

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
    }

    public void setFlyWheelSpeed(double targetSpeed) {
        flywheelMotor.set(
            flywheelController.calculate(
                flywheelEncoder.getVelocity(), 
                targetSpeed
            )
        ); 
    }


}
