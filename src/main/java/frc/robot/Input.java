package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Input {
    private static final XboxController opController = new XboxController(1); 

    public static boolean startIntaking() {
        return opController.getAButton(); 
    }

    public static boolean pivotIntake() {
        return opController.getRightBumperButton(); 
    }

  
}
