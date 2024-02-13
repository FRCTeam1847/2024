// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightsSubsystem;

public class ShootingAnimationCommand extends Command {
  private LightsSubsystem lightsSubsystem;

  int OnIndex;
  int counter;

  /** Creates a new ShootingAnimation. */
  public ShootingAnimationCommand(LightsSubsystem _lightsSubsystem) {
    lightsSubsystem = _lightsSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_lightsSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    OnIndex = 0;
    counter = 0;
    lightsSubsystem.LightsOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (counter > 1) {
      // counter = 0;
      if (OnIndex < 19) {
        // Left side
        lightsSubsystem.m_ledBuffer.setLED(OnIndex, lightsSubsystem.offColor);
        lightsSubsystem.m_ledBuffer.setLED(OnIndex + 1, lightsSubsystem.greenColor);

        // // Right side
        lightsSubsystem.m_ledBuffer.setLED(lightsSubsystem.RightLights - OnIndex, lightsSubsystem.offColor);
        lightsSubsystem.m_ledBuffer.setLED(lightsSubsystem.RightLights - (OnIndex + 1), lightsSubsystem.greenColor);
        OnIndex++;
      } else {
        // Left side
        lightsSubsystem.m_ledBuffer.setLED(OnIndex, lightsSubsystem.offColor);
        lightsSubsystem.m_ledBuffer.setLED(0, lightsSubsystem.offColor);
        // Right side
        lightsSubsystem.m_ledBuffer.setLED(lightsSubsystem.RightLights - OnIndex, lightsSubsystem.offColor);
        lightsSubsystem.m_ledBuffer.setLED(lightsSubsystem.RightLights, lightsSubsystem.greenColor);

        OnIndex = 0;
      }

    // }
    // counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    for (var i = 0; i < 19; i++) {
      lightsSubsystem.m_ledBuffer.setLED(i, lightsSubsystem.redColor);
      lightsSubsystem.m_ledBuffer.setLED(lightsSubsystem.RightLights - i, lightsSubsystem.redColor);
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
