package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Input {

    private static final XboxController driveController = new XboxController(0);
    private static final XboxController operatorController = new XboxController(1);

    public static XboxController getDriveController() {
        return driveController;
    }

    public static XboxController getOperatorController() {
        return operatorController;
    }
    

    // temp inputs
    private static final double deadzone = 0.5;
    
    public static double getIntakeJointSpeed() {
        return operatorController.getRightY();
    }

    public static boolean lowerIntake() {
        return operatorController.getRightY() > deadzone;
    }

    public static boolean raiseIntake() {
        return operatorController.getRightY() < -deadzone;
    }
}