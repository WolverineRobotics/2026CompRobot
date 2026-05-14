package frc.robot.subsystems;


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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
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
                new SysIdRoutine.Mechanism(this::setVoltage, 
                 log -> {
                 log.motor("flywheel-motor").voltage(getFlywheelVoltage()).angularPosition(getFlywheelAngle()).angularVelocity(getFlyWheelVelocity())

            }, this)
            );

        indexerConfig = new SparkMaxConfig(); 
        indexerConfig.smartCurrentLimit(ShooterConstants.indexerCurrentLimit, ShooterConstants.indexerCurrentLimit);
        indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 

        flywheelConfig = new SparkMaxConfig(); 
        flywheelConfig.inverted(ShooterConstants.flywheelInverted); 
        flywheelConfig.smartCurrentLimit(ShooterConstants.flywheelCurrentLimit, ShooterConstants.flywheelCurrentLimit);
        flywheelMotor.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setFlyWheelSpeed(double targetSpeed) {
        SmartDashboard.putNumber("Target Speed", targetSpeed);
        double ff = flywheelFeedforward.calculate(targetSpeed); 
        double pid = flywheelPIDController.calculate(getFlyWheelVelocity().abs(RadiansPerSecond), targetSpeed); 
        flywheelMotor.setVoltage(
            ff + pid           
        );

        SmartDashboard.putNumber("FeedForward Output", ff);
        SmartDashboard.putNumber("PID Output",  flywheelPIDController.calculate(
                flywheelEncoder.getVelocity(),
                targetSpeed
            ));
    }

    public void ControllerDebugger() {
        setFlyWheelSpeed(-500);
    }

    public void setVoltage(Voltage targetVoltage) {
        flywheelMotor.setVoltage(targetVoltage);
    }


   
    public void spinIndexer(double speed) {
        indexerMotor.set(speed);
    }

    public void spinFlywheel(double speed) {
        flywheelMotor.set(speed); 
    }

    public AngularVelocity getFlyWheelVelocity() {
        return RadiansPerSecond.of(Units.rotationsPerMinuteToRadiansPerSecond(flywheelEncoder.getVelocity())); 
    }

    public Angle getFlywheelAngle() {
        return Radians.of(flywheelEncoder.getPosition()); 
    }

    public Voltage getFlywheelVoltage() {
        return Volts.of(flywheelMotor.getBusVoltage()); 
    }

    public double getTargetVelocity(double range) {
        double tantheta = Math.tan(ShooterConstants.shooterAngle);
        double costheta = Math.cos(ShooterConstants.shooterAngle);

        return Math.sqrt(
          (-9.81 * range * range) / 
          (2 * costheta * costheta * ((ShooterConstants.hubHeight - ShooterConstants.shooterHeight) - (range * tantheta))
        )) / ShooterConstants.flywheelRadius;
    }

    @Override 
    public void periodic() {
        SmartDashboard.putNumber("Flywheel Velocity RPM", getFlyWheelVelocity().magnitude());
        SmartDashboard.putNumber("Flywheel Current Output", flywheelMotor.getOutputCurrent()); 
    }

    public double getTestSpeed() {
        return testableRPM.getDouble(0); 
    }

    public Command getQuasiStaticTest(SysIdRoutine.Direction direction) {
        return tuningRoutine.quasistatic(direction); 
    }

    public Command getDynamicTest(SysIdRoutine.Direction direction) {
        return tuningRoutine.dynamic(direction);
    }








}
