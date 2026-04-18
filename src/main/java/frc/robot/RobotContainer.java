// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.OuttakeCommand;
import frc.robot.commands.intake.PivotCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.shooter.ShootConstantCommand;
import frc.robot.commands.shooter.ShootVariableCommand;
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

    NamedCommands.registerCommand("Shoot Fuel", new ShootConstantCommand(m_ShooterSubsystem, 0));
    NamedCommands.registerCommand("Pivot Intake", new PivotCommand(m_IntakeSubsystem));

  }

  private void configureBindings() {
    // Input.rightBumper.whileTrue(new ShootCommand(m_ShooterSubsystem, m_DriveSubsystem));
    
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Test Auto");
  }

  public void teleopSequence() {
    if (Input.spinFlywheel()) {
      CommandScheduler.getInstance().schedule(new ShootConstantCommand(m_ShooterSubsystem, ShooterConstants.flywheelSpeed));
    }

    if (Input.startIntaking()) {
      CommandScheduler.getInstance().schedule(new IntakeCommand(m_IntakeSubsystem));
    }
    if (Input.startOuttaking()) {
      CommandScheduler.getInstance().schedule(new OuttakeCommand(m_IntakeSubsystem));
    }

    if (Input.pivotIntake()) {
      CommandScheduler.getInstance().schedule(new PivotCommand(m_IntakeSubsystem));
    }

    if (Input.funnelFuel()) {
      CommandScheduler.getInstance().schedule(new ShootConstantCommand(m_ShooterSubsystem, ShooterConstants.flywheelFunnelSpeed));
    }

   
    
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putNumber("Input Forward", Input.getVertical());
    SmartDashboard.putBoolean("Intaking", Input.startIntaking());
    SmartDashboard.putBoolean("Input Shooting", Input.spinFlywheel());

  
  }
}
