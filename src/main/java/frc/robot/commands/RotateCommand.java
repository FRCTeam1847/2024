// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainSubsystem;

public class RotateCommand extends Command {

  DriveTrainSubsystem driveTrainSubsystem;
  double speed = 0.5;
  double angle = 0;

  /** Creates a new RotateCommand. */
  public RotateCommand(DriveTrainSubsystem _driveTrainSubsystem, double _angle) {
    driveTrainSubsystem = _driveTrainSubsystem;
    angle = _angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrainSubsystem.gyro.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = driveTrainSubsystem.gyro.getAngle();
    if (currentAngle < angle) {
      driveTrainSubsystem.ArcadeDrive(0, speed, false);
    } else if (angle > angle) {
      driveTrainSubsystem.ArcadeDrive(0, -speed, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveTrainSubsystem.gyro.getAngle() < angle || driveTrainSubsystem.gyro.getAngle() > angle;
  }
}
