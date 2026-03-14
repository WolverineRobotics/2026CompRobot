package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Input {
  
    private static final XboxController opController = new XboxController(1); 
    private static final XboxController driveController = new XboxController(0);


    public static boolean spinFlywheel() {
        return opController.getRightBumperButton(); 
    }

    public static boolean endFlywheel() {
        return opController.getBButtonReleased();
    }
  
    public static double getVertical() {
        return driveController.getLeftY(); 
    }

    public static double getHorizontal() {
        return driveController.getLeftX(); 
    }

    public static double getRotation() {
        return driveController.getRightX(); 
    }
   
    public static boolean startIntaking() {
        return opController.getAButton(); 
    }

    public static double pivotIntake() {
        return opController.getRightY() * 0.1; 
    }

  
}
