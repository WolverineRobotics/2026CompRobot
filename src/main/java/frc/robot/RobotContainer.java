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

import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class RobotContainer {

  private final ShooterSubsystem m_ShooterSubsystem; 
  private final DriveSubsystem m_DriveSubsystem; 
  private final IntakeSubsystem m_IntakeSubsystem;


  public RobotContainer() {
    m_ShooterSubsystem = new ShooterSubsystem(); 
    configureBindings();
    m_DriveSubsystem = new DriveSubsystem(); 
    CommandScheduler.getInstance().run();
    m_IntakeSubsystem = new IntakeSubsystem(); 

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
    if (Input.startIntaking()) {
      CommandScheduler.getInstance().schedule(new IntakeCommand(m_IntakeSubsystem));
    }

    if (Input.pivotIntake() != 0) {
      CommandScheduler.getInstance().schedule(new PivotCommand(m_IntakeSubsystem));
    }

    SmartDashboard.putData(CommandScheduler.getInstance());

  
  }
}
