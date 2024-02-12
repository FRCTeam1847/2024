// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightsSubsystem;

public class NoteCollectedAnimationCommand extends Command {
  private LightsSubsystem lightsSubsystem;
  /** Creates a new NoteCollectedCommand. */
  public NoteCollectedAnimationCommand(LightsSubsystem _lightsSubsystem) {
    lightsSubsystem = _lightsSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_lightsSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lightsSubsystem.LightsOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for (var i = 0; i < 19; i++) {
      lightsSubsystem.m_ledBuffer.setLED(i, lightsSubsystem.greenColor);
      lightsSubsystem.m_ledBuffer.setLED(lightsSubsystem.RightLights-i, lightsSubsystem.greenColor);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
