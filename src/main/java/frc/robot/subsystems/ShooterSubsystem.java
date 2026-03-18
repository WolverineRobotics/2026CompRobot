package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem  extends SubsystemBase {

    private final SparkMax flywheelMotor; 
    private final SparkMax indexerMotor; 

    private final SparkMaxConfig indexerConfig; 
    private final SparkMaxConfig flywheelConfig; 

    private final RelativeEncoder flywheelEncoder; 

    private final PIDController flywheelPIDController; 
    private final SimpleMotorFeedforward flywheelFeedforward; 


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
        

        flywheelPIDController = new PIDController(
            ShooterConstants.flyWheelKp, 
            ShooterConstants.flyWheelKi,
            ShooterConstants.flyWheelKd
        ); 

        flywheelFeedforward = new SimpleMotorFeedforward(
            ShooterConstants.flyWheelKs,
            ShooterConstants.flyWheelKv     
        );

        indexerConfig = new SparkMaxConfig(); 
        indexerConfig.smartCurrentLimit(ShooterConstants.indexerCurrentLimit, ShooterConstants.indexerCurrentLimit);
        indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 

        flywheelConfig = new SparkMaxConfig(); 
        flywheelConfig.inverted(ShooterConstants.flywheelInverted); 
        flywheelMotor.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setFlyWheelSpeed(double targetSpeed) {
        SmartDashboard.putNumber("Target Speed", targetSpeed);
        flywheelMotor.setVoltage(
            flywheelFeedforward.calculate(targetSpeed) 
            // flywheelPIDController.calculate(
            //     getFlyWheelVelocity(),
            //     targetSpeed
            // ) 
            
        );

        SmartDashboard.putNumber("FeedForward Output", flywheelFeedforward.calculate(targetSpeed));
        SmartDashboard.putNumber("PID Output",  flywheelPIDController.calculate(
                flywheelEncoder.getVelocity(),
                targetSpeed
            ));
    }

    public void ControllerDebugger() {
        setFlyWheelSpeed(-500);
    }

    public void KsDebugger() {
        flywheelMotor.setVoltage(12);
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

    public double getTargetVelocity(double range) {
        double tantheta = Math.tan(ShooterConstants.shooterAngle);
        double costheta = Math.cos(ShooterConstants.shooterAngle);

        return Math.sqrt(
        (9.81 * range * range) /
        (2 * costheta * costheta * (ShooterConstants.hubHeight - (range * tantheta)))
        ) / ShooterConstants.flywheelRadius;
    }

    @Override 
    public void periodic() {
        SmartDashboard.putNumber("Flywheel Velocity radps", getFlyWheelVelocity());
    }


}
