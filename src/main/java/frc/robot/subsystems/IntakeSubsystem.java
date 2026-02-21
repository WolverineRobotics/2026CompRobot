package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;


public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax rPiviotMotor;
    private final SparkMax lPiviotMotor;
    private final SparkMax IntakeMotor;
    private final DigitalInput LimitAng; //Limit angle
    private final SparkMaxConfig rPiviotConfig;
    private final SparkMaxConfig lPiviotConfig;

    public IntakeSubsystem() {
        rPiviotMotor = new SparkMax(0, MotorType.kBrushless);
        lPiviotMotor = new SparkMax(1, MotorType.kBrushless);
        IntakeMotor = new SparkMax(3, MotorType.kBrushless);
        LimitAng = new DigitalInput(0);

        //figuring out the controller code linked to run the intake e
    }


    public void piviotintake() {
        /* 
        get the rPiviotMotor follow the lPiviotMotor down till the DigitalInput says true in the set angle
*/
         //where I tried to let the motors move together


    }

}



