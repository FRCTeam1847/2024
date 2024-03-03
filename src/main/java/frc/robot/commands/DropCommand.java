// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;

public class DropCommand extends Command {
  LauncherSubsystem launcherSubsystem;

  double maxSpeed = 0.5;
  double feedSpeed = 0.5;
  double timeoutTime = 2;
  double waitTime = 1;

  // Only need this if we have to use time stuff
  private Timer localTimer = new Timer();

  /** Creates a new ShootCommand. */
  public DropCommand(LauncherSubsystem _launcherSubsystem) {
    launcherSubsystem = _launcherSubsystem;
    addRequirements(_launcherSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (launcherSubsystem.getBottomSwitchValue()) {
      launcherSubsystem.m_feed.set(maxSpeed);
      localTimer.reset();
      localTimer.start();
    } else {
      System.out.println("Missing Note");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (launcherSubsystem.m_launch.getMotorOutputPercent() >= maxSpeed && localTimer.get() > waitTime) {
      launcherSubsystem.m_feed.set(feedSpeed);
    } else {
      System.out.println(String.format("Waiting for speed! Time: %,.2f", localTimer.get()));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcherSubsystem.StopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !launcherSubsystem.getBottomSwitchValue() || localTimer.get() > timeoutTime;
  }
}
