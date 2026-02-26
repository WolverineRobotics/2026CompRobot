package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax rPivotMotor;
    private final SparkMax lPivotMotor;
    private final SparkMax IntakeMotor;
    private final DigitalInput LimitAng; //Limit angle
    private final SparkMaxConfig rPivotConfig;
    private final SparkMaxConfig lPivotConfig;
    

    public IntakeSubsystem() {
        rPivotMotor = new SparkMax(0, MotorType.kBrushless);
        lPivotMotor = new SparkMax(1, MotorType.kBrushless);
        IntakeMotor = new SparkMax(3, MotorType.kBrushless);
        LimitAng = new DigitalInput(0);

        /* Move Pivot motors at the same time (up and down)
         * One motor to get the balls into the hopper
         * A limit angle (can vary) to not break the pivot motors and the robot itself
         * Lower intake/raise intake
         * intake balls
         * get state
         */
    }


    public void pivotintake() {    //Motors move together, which I need to learn how to do that in the first place
        /* 
        get the rPiviotMotor follow the lPiviotMotor down till the DigitalInput says true in the set angle
*/
      rPivotMotor.set(1);
      lPivotMotor.set(1);

      if (!LimitAng.get()) {
        rPivotMotor.set(IntakeConstants.intakeSpeed); // this will stop both the Pivot motors when the limit switch gets triggered
        lPivotMotor.set(IntakeConstants.intakeSpeed);
      }
      rPivotMotor.set(0);
      lPivotMotor.set(0);

    }


    public void ingamePiece(){
        IntakeMotor.set(1);
    }

    public void digitalInput(){
        
    }


}



