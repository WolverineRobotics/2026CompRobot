package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Input {

    private static final XboxController driveController = new XboxController(0);

    public static double getVertical() {
        return driveController.getLeftY(); 
    }

    public static double getHorizontal() {
        return driveController.getLeftX() * -1; 
    }

    public static double getRotation() {
        return driveController.getRightX(); 
    }
    
}
