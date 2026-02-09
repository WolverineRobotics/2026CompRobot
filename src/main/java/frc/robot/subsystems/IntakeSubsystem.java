package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class IntakeSubsystem extends SubsystemBase {

    public IntakeSubsystem() {
        Spark mSpark;
        mSpark = new Spark(0);
        //figuring out the controller code linked to run the intake
        mSpark.set(1.5);

    }

}
