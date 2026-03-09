package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
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
    

    public IntakeSubsystem() {
        rightPivotMotor = new SparkMax(IntakeConstants.rightPivotID, MotorType.kBrushless);
        leftPivotMotor = new SparkMax(IntakeConstants.leftPivotID, MotorType.kBrushless);
        IntakeMotor = new SparkMax(IntakeConstants.intakeMotorCanID, MotorType.kBrushless);
        leftPivotConfig = new SparkMaxConfig(); 
        leftPivotConfig.follow(rightPivotMotor); 
        leftPivotMotor.configure(leftPivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

       
    }


    public void pivotintake() {    

      rightPivotMotor.set(IntakeConstants.pivotSpeed);
      if (atBumpers()) {
        rightPivotMotor.set(0); 
      }

    
    }


    public void intakeGamepiece(){ //game piece goes in
        IntakeMotor.set(IntakeConstants.intakeSpeed);
    }

    public boolean atBumpers() {
      return (rightPivotMotor.getOutputCurrent() >= IntakeConstants.stressedIntakeCurrentDraw);
    }



}



