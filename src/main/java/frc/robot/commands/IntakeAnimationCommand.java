// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightsSubsystem;

public class IntakeAnimationCommand extends Command {
  private LightsSubsystem lightsSubsystem;
  int counter;
  int OnIndex;

  /** Creates a new IntakeCommand. */
  public IntakeAnimationCommand(LightsSubsystem _lightsSubsystem) {
    lightsSubsystem = _lightsSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_lightsSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    OnIndex = 19;
    lightsSubsystem.LightsOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (counter > 1) {
      counter = 0;
      if (OnIndex > 0) {
        // Left side
        lightsSubsystem.m_ledBuffer.setLED(OnIndex, lightsSubsystem.offColor);
        lightsSubsystem.m_ledBuffer.setLED(OnIndex - 1, lightsSubsystem.goldColor);

        // // Right side
        lightsSubsystem.m_ledBuffer.setLED(lightsSubsystem.RightLights - OnIndex, lightsSubsystem.offColor);
        lightsSubsystem.m_ledBuffer.setLED(lightsSubsystem.RightLights - (OnIndex - 1), lightsSubsystem.goldColor);
        OnIndex--;
      } else {
        // Left side
        lightsSubsystem.m_ledBuffer.setLED(OnIndex, lightsSubsystem.offColor);
        lightsSubsystem.m_ledBuffer.setLED(19, lightsSubsystem.goldColor);
        // Right side
        lightsSubsystem.m_ledBuffer.setLED(lightsSubsystem.RightLights, lightsSubsystem.offColor);
        lightsSubsystem.m_ledBuffer.setLED(lightsSubsystem.RightLights - 19, lightsSubsystem.goldColor);

        OnIndex = 19;
      }

    }
    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    for (var i = 0; i < 19; i++) {
      lightsSubsystem.m_ledBuffer.setLED(i, lightsSubsystem.greenColor);
      lightsSubsystem.m_ledBuffer.setLED(lightsSubsystem.RightLights-i, lightsSubsystem.greenColor);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
