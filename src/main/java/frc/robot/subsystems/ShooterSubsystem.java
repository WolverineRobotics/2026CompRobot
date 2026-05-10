package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem  extends SubsystemBase {

    private final SparkFlex flywheelMotor; 
    private final SparkMax indexerMotor; 

    private final SparkMaxConfig indexerConfig; 
    private final SparkMaxConfig flywheelConfig; 

    private final RelativeEncoder flywheelEncoder; 

    private final PIDController flywheelPIDController; 
    private final SimpleMotorFeedforward flywheelFeedforward; 
    private final GenericEntry testableRPM; 

    private final SysIdRoutine tuningRoutine; 
    
    
        public ShooterSubsystem() {
            flywheelMotor = new SparkFlex(
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
    
            testableRPM = Shuffleboard.getTab("Tuning").add("RPM", 0).getEntry();

            tuningRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(this::setFlyWheelVoltage, 
                (log) -> {
                    log.motor("Flywheel")
                        .voltage(Volts.of(getAppliedFlywheelVoltage()))
                        .angularPosition(Radians.of(Units.rotationsToRadians(getFlywheelPosition())))
                        .angularVelocity(RadiansPerSecond.of(Units.rotationsPerMinuteToRadiansPerSecond(getFlyWheelVelocity())));
                }, 
                this
                )
            );

        indexerConfig = new SparkMaxConfig(); 
        indexerConfig.smartCurrentLimit(ShooterConstants.indexerCurrentLimit, ShooterConstants.indexerCurrentLimit);
        indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 

        flywheelConfig = new SparkMaxConfig(); 
        flywheelConfig.inverted(ShooterConstants.flywheelInverted); 
        flywheelConfig.smartCurrentLimit(ShooterConstants.flywheelCurrentLimit, ShooterConstants.flywheelCurrentLimit);
        flywheelMotor.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override 
    public void periodic() {
        SmartDashboard.putNumber("Flywheel Velocity RPM", getFlyWheelVelocity());
        SmartDashboard.putNumber("Flywheel Current Output", flywheelMotor.getOutputCurrent()); 
    }

    

    /*
     * FLYWHEEL CODE 
     */

    public void spinFlywheel(double speed) {
        flywheelMotor.set(speed); 
    }

    public double getFlyWheelVelocity() {
        return flywheelEncoder.getVelocity(); 
    }

    public double getFlywheelPosition() {
        return flywheelEncoder.getPosition();
    }

    public double getAppliedFlywheelVoltage() {
        return flywheelMotor.getBusVoltage(); 
    }

    public double getTargetVelocity(double range) {
        double tantheta = Math.tan(ShooterConstants.shooterAngle);
        double costheta = Math.cos(ShooterConstants.shooterAngle);

        return Math.sqrt(
          (-9.81 * range * range) / 
          (2 * costheta * costheta * ((ShooterConstants.hubHeight - ShooterConstants.shooterHeight) - (range * tantheta))
        )) / ShooterConstants.flywheelRadius;
    }

    public void setFlyWheelVoltage(Voltage targetVoltage) {
        flywheelMotor.setVoltage(targetVoltage);
    }

    public void setFlyWheelSpeed(double targetSpeed) {
        SmartDashboard.putNumber("Target Speed", targetSpeed);
        double ff = flywheelFeedforward.calculate(targetSpeed); 
        flywheelMotor.setVoltage(
            ff

            
        );

        SmartDashboard.putNumber("FeedForward Output", ff);
        SmartDashboard.putNumber("PID Output",  flywheelPIDController.calculate(
                flywheelEncoder.getVelocity(),
                targetSpeed
            ));
    }

    /*
     * INDEXER CODE
     */

    public void spinIndexer(double speed) {
        indexerMotor.set(speed);
    }

    /*
     * DEBUG CODE
     */

    public void ControllerDebugger() {
        setFlyWheelSpeed(-500);
    }

    public void KsDebugger() {
        flywheelMotor.setVoltage(6);
    } 
   

      public double getTestSpeed() {
        return testableRPM.getDouble(0); 
    }

    public Command QuasiStaticTest(SysIdRoutine.Direction direction) {
        return tuningRoutine.quasistatic(direction); 
    }


    public Command DynamicTest(SysIdRoutine.Direction direction) {
        return tuningRoutine.dynamic(direction); 
    }







}
