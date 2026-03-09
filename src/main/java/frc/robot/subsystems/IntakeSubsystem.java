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
import frc.robot.Constants.pivotrlMotors;

import frc.robot.Input;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax rPivotMotor;
    private final SparkMax lPivotMotor;
    private final SparkMax IntakeMotor;
    private final DigitalInput LimitAng; //Limit angle
    // private  SparkMaxConfig rPivotConfig;  Not used right now I think
    // private  SparkMaxConfig lPivotConfig;


    

    public IntakeSubsystem() {
        rPivotMotor = new SparkMax(pivotrlMotors.rpivotMotor, MotorType.kBrushless);
        lPivotMotor = new SparkMax(pivotrlMotors.lpivotMotor, MotorType.kBrushless);
        IntakeMotor = new SparkMax(IntakeConstants.intakeMotorCanID, MotorType.kBrushless);
        LimitAng = new DigitalInput(IntakeConstants.LimitAngID);

        /* Move Pivot motors at the same time (up and down)
         * One motor to get the balls into the hopper
         * A limit angle (can vary) to not break the pivot motors and the robot itself
         * Lower intake/raise intake
         * intake balls
         * get state
         */

        setDefaultCommand();
    }


    public void pivotintake() {  //Intake falls to intaking motion to get the balls
      //
      //Motors move together, which I need to learn how to do that in the first place
        /* 
        get the rPiviotMotor follow the lPiviotMotor down till the DigitalInput says true in the set angle
*/
      
      rPivotMotor.set(pivotrlMotors.pivotspeed);
      lPivotMotor.set(pivotrlMotors.pivotspeed);

// Try to use different digital inputs for work which I'm too sure how yet, 

      if (!LimitAng.get()) {
        rPivotMotor.set(pivotrlMotors.rpivotMotor); // this will stop both the Pivot motors when the limit switch gets triggered
        lPivotMotor.set(pivotrlMotors.lpivotMotor); //isn't fully developed yet
      }
      rPivotMotor.set(0);
      lPivotMotor.set(0); //stops both of the motors

    }

    public void pivotout(){ //Intake goes back into position in theory
      // need to get some more tests


      //Extra measures
      if (!LimitAng.get()){
        rPivotMotor.set(pivotrlMotors.rpivotMotor);
        lPivotMotor.set(pivotrlMotors.lpivotMotor);
      }
      rPivotMotor.set(0);
      lPivotMotor.set(0);

    }


    public void ingamePiece(){ //game piece goes in
        IntakeMotor.set(IntakeConstants.intakeSpeed);
    }

    public void digitalInput(){
        
    }




// temp raw command
    public void setIntakeMotor(double speed) {
        lPivotMotor.set(speed);
        rPivotMotor.set(speed);
    }

    public boolean atBottom(){
        return LimitAng.get();
    }

    public boolean atTop(){
        return !LimitAng.get();
    }


  public void setDefaultCommand() {
    this.setDefaultCommand(new edu.wpi.first.wpilibj2.command.InstantCommand(
      () -> {
        if(Input.lowerIntake()) {
          setIntakeMotor(0.5);
        } else if (Input.raiseIntake()) {
          setIntakeMotor(-0.5);
        } else {
          setIntakeMotor(0);
        }
      },
      this
    ));
  }



  //example of where to put safety
  private boolean atBottomFlag = false;
  private boolean atTopFlag = false;

  @Override
  public void periodic() {

    // if(LimitAng.get()) {
    //   atBottomFlag = true;
    // }

    // if(lPivotMotor.getEncoder().getPosition() > 90 && rPivotMotor.getEncoder().getPosition() > 90) {
    //   atTopFlag = false;
    // }

    // if(rPivotMotor.getEncoder().getPosition() < 0 && lPivotMotor.getEncoder().getPosition() < 0) {
    //   atBottomFlag = false;
    // }


    // if(atBottomFlag) {
    //   rPivotMotor.set(0);
    //   lPivotMotor.set(0);
    // }



    SmartDashboard.putNumber("leftPivotAngle", lPivotMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("rightPivotAngle", rPivotMotor.getEncoder().getPosition());
    
    
  }

}



