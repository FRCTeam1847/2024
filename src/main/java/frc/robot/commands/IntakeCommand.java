// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootingSubsystem;

public class IntakeCommand extends Command {
  ShootingSubsystem shootingSubsystem;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(ShootingSubsystem _shootingSubsystem) {
    shootingSubsystem = _shootingSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_shootingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      shootingSubsystem.IntakeNote();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shootingSubsystem.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
