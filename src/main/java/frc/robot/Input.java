package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Input {
  
    private static final XboxController opController = new XboxController(1); 
    private static final XboxController driveController = new XboxController(0);

    public static final JoystickButton xButton = new JoystickButton(opController, XboxController.Button.kX.value);


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
 
    public static double getVertical() {
        return deadband(driveController.getLeftY()); 
    }

    public static double getHorizontal() {
        return deadband(driveController.getLeftX()); 
    }

    public static double getRotation() {
        return deadband(driveController.getRightX()); 
    }

    public static double getPivotSpeed() {
        return deadband(opController.getRightY() * 0.25); 
    }
   
    public static boolean startIntaking() {
        return opController.getLeftBumperButton(); 
    }
    public static boolean startOuttaking() {
        return opController.getBButton(); 
    }

    public static boolean lowerIntake() {
        return opController.getYButton(); 
    }

    public static boolean spinIndexer() {
        return opController.getAButton(); 
    }

    public static boolean funnelFuel() {
        return opController.getXButton(); 
    }

    

  
}
