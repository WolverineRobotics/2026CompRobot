package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Input {
  
    private static final XboxController opController = new XboxController(1); 
    private static final XboxController driveController = new XboxController(0);

    public static final JoystickButton rightBumper = new JoystickButton(opController, XboxController.Button.kRightBumper.value);


    private static double deadband(double input) {
        if (Math.abs(input) < 0.075) {
            return 0;
        }
        else {
            return input;
        }
    }


    public static boolean spinFlywheel() {
        return opController.getRightBumperButton(); 
    }

    public static boolean endFlywheel() {
        return opController.getBButtonReleased();
    }
  
    public static double getVertical() {
        return deadband(driveController.getLeftY()); 
    }

    public static double getHorizontal() {
        return deadband(driveController.getLeftX()); 
    }

    public static double getRotation() {
        return deadband(driveController.getRightX()); 
    }
   
    public static boolean startIntaking() {
        return opController.getAButton(); 
    }
    public static boolean startOuttaking() {
        return opController.getBButton(); 
    }


    public static boolean pivotIntake() {
        return opController.getLeftBumperButton(); 
    }

  
}
