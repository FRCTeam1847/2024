// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;

public class IntakeCommand extends Command {
  LauncherSubsystem launcherSubsystem;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(LauncherSubsystem _launcherSubsystem) {
    launcherSubsystem = _launcherSubsystem;
    addRequirements(_launcherSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (launcherSubsystem.getTopSwitchValue()) {
      launcherSubsystem.m_feed.set(-0.25);
      launcherSubsystem.m_launch.set(-0.5);
    } else {
      System.out.println("No Note is present");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcherSubsystem.StopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return launcherSubsystem.getBottomSwitchValue();
  }
}
