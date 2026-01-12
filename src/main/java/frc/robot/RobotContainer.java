// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class RobotContainer {
  
  private final DriveSubsystem m_DriveSubsystem; 

  public RobotContainer() {
    configureBindings();
    m_DriveSubsystem = new DriveSubsystem(); 
    CommandScheduler.getInstance().run();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void teleopSequence() {
    SmartDashboard.putData(CommandScheduler.getInstance());
  }
}
