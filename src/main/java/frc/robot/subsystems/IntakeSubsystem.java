package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax rightPivotMotor;
    private final SparkMax leftPivotMotor;
    private final SparkMax IntakeMotor;
    private final SparkMaxConfig leftPivotConfig;
    private final SparkMaxConfig rightPivotConfig; 
    

    public IntakeSubsystem() {
        rightPivotMotor = new SparkMax(IntakeConstants.rightPivotID, MotorType.kBrushless);
        leftPivotMotor = new SparkMax(IntakeConstants.leftPivotID, MotorType.kBrushless);
        IntakeMotor = new SparkMax(IntakeConstants.intakeMotorID, MotorType.kBrushless);

        leftPivotConfig = new SparkMaxConfig(); 
        rightPivotConfig = new SparkMaxConfig(); 

        leftPivotConfig.follow(rightPivotMotor); 
        leftPivotConfig.idleMode(IdleMode.kBrake); 
        leftPivotMotor.configure(leftPivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        rightPivotConfig.idleMode(IdleMode.kBrake); 
        rightPivotMotor.configure(rightPivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

       
    }


    public void pivotintake() {    

      rightPivotMotor.set(IntakeConstants.pivotSpeed);
      if (atBumpers()) {
        rightPivotMotor.set(0); 
      }
      
    
    }


    public void spinRoller(double speed){ //game piece goes in
        IntakeMotor.set(speed);
    }

    public boolean atBumpers() {
      return (rightPivotMotor.getOutputCurrent() >= IntakeConstants.stressedIntakeCurrentDraw);
    }

    @Override
    public void periodic() {
      SmartDashboard.putNumber("Intake Speed", IntakeMotor.getEncoder().getPosition());
      SmartDashboard.putNumber("Pivot Current Draw", rightPivotMotor.getOutputCurrent()); 
    }



}



