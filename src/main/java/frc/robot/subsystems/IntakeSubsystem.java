package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;

public class IntakeSubsystem extends SubsystemBase {
        private final Spark mSpark = new Spark(0);
        private final XboxController iController = new XboxController(0);
    public IntakeSubsystem() {


        //figuring out the controller code linked to run the intake
    }
}



