// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

  private final ShooterSubsystem m_ShooterSubsystem; 

  public RobotContainer() {
    m_ShooterSubsystem = new ShooterSubsystem(); 
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void teleopSequence() {
    if (Input.spinFlywheel()) {
      CommandScheduler.getInstance().schedule(new ShootCommand(m_ShooterSubsystem));
    }

    if (Input.endFlywheel()) {
      CommandScheduler.getInstance().cancel(new ShootCommand(m_ShooterSubsystem));
    }
  }
}
