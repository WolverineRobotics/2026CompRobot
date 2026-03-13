// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
  private final IntakeSubsystem m_IntakeSubsystem; 

  public RobotContainer() {
    configureBindings();
    m_IntakeSubsystem = new IntakeSubsystem(); 
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
  
  public void teleopSequence() {
    SmartDashboard.putData(CommandScheduler.getInstance());
    if (Input.startIntaking()) {
      CommandScheduler.getInstance().schedule(new IntakeCommand(m_IntakeSubsystem));
    }

    if (Input.pivotIntake()) {
      CommandScheduler.getInstance().schedule(new PivotCommand(m_IntakeSubsystem));
    }
  }
}
