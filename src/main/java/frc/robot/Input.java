package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Input {
    
    private static final XboxController opController = new XboxController(1); 

    public static boolean spinFlywheel() {
        return opController.getBButton(); 
    }

    public static boolean endFlywheel() {
        return opController.getBButtonReleased();
    }
}
