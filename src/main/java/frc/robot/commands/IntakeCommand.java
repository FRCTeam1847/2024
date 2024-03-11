// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LightsSubsystem;

public class IntakeCommand extends Command {
  private LauncherSubsystem launcherSubsystem;
  private LightsSubsystem lightsSubsystem;
  int counter;
  int OnIndex;
  double timeoutTime = 0.5;

  // Only need this if we have to use time stuff
  private Timer localTimer = new Timer();

  /** Creates a new IntakeCommand. */
  public IntakeCommand(LauncherSubsystem _launcherSubsystem, LightsSubsystem _lightsSubsystem) {
    launcherSubsystem = _launcherSubsystem;
    lightsSubsystem = _lightsSubsystem;
    addRequirements(_launcherSubsystem, _lightsSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!launcherSubsystem.noteSwitch.get()) {
      counter = 0;
      OnIndex = 19;
      lightsSubsystem.LightsOff();
      launcherSubsystem.setFeedWheel(-0.2);
      launcherSubsystem.setLaunchWheel(-0.4);
      localTimer.reset();
      localTimer.start();
    } else {
      System.out.println("No Note");
    }
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
    launcherSubsystem.StopMotors();
    if (launcherSubsystem.noteSwitch.get()) {
      for (var i = 0; i < 19; i++) {
        lightsSubsystem.m_ledBuffer.setLED(i, lightsSubsystem.greenColor);
        lightsSubsystem.m_ledBuffer.setLED(lightsSubsystem.RightLights - i, lightsSubsystem.greenColor);
      }
    }
    else{
      for (var i = 0; i < 19; i++) {
        lightsSubsystem.m_ledBuffer.setLED(i, lightsSubsystem.redColor);
        lightsSubsystem.m_ledBuffer.setLED(lightsSubsystem.RightLights - i, lightsSubsystem.redColor);
      }
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return localTimer.get() >= timeoutTime;
  }
}
