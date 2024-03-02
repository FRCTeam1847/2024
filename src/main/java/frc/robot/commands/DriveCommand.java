// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveCommand extends Command {
  DriveTrainSubsystem driveTrainSubsystem;
  /** Creates a new DriveCommand. */
  public DriveCommand(DriveTrainSubsystem _driveTrainSubsystem) {
   driveTrainSubsystem = _driveTrainSubsystem;
   addRequirements(_driveTrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftX = Math.pow( RobotContainer.m_driverController.getLeftX(), 3);
    double leftY = Math.pow( RobotContainer.m_driverController.getLeftY(), 3);
    driveTrainSubsystem.ArcadeDrive(leftX, leftY);
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
