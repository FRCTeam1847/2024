// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveBackwardsDistance extends Command {
  DriveTrainSubsystem driveTrainSubsystem;

  double angle;
  double speed = -0.5;
  double maxAngle = 1;
  double distance = 0.0;

  /** Creates a new DriveStraightDistance. */
  public DriveBackwardsDistance(DriveTrainSubsystem _driveTrainSubsystem, double inches) {
    driveTrainSubsystem = _driveTrainSubsystem;
    distance = inches;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrainSubsystem.gyro.reset();
    driveTrainSubsystem.leftEncoder.reset();
    driveTrainSubsystem.rightEncoder.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle = driveTrainSubsystem.gyro.getAngle();
    if (angle > maxAngle) {
      driveTrainSubsystem.ArcadeDrive(speed, speed, false);
    } else if (angle < -maxAngle) {
      driveTrainSubsystem.ArcadeDrive(speed, -speed, false);
    } else {
      driveTrainSubsystem.ArcadeDrive(speed, 0, false);
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
    return driveTrainSubsystem.leftEncoder.getDistance() < -distance ||  driveTrainSubsystem.rightEncoder.getDistance() < -distance;
  }
}
