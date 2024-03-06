// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LightsSubsystem;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command DriveBackwardsInches(DriveTrainSubsystem subsystem, double inches) {
    return Commands.sequence(new DriveBackwardsDistance(subsystem, inches));
  }

  public static Command DriveInchesRotate(DriveTrainSubsystem subsystem, double inches, double degrees) {
    return Commands.sequence(new DriveBackwardsDistance(subsystem, inches), new RotateCommand(subsystem, degrees));
  }

  public static Command Rotate(DriveTrainSubsystem subsystem, double degrees) {
    return Commands.sequence(new RotateCommand(subsystem, degrees));
  }

  public static Command ShootRotateDriveBackwards(DriveTrainSubsystem subsystem, LauncherSubsystem launcherSubsystem,
      LightsSubsystem lightsSubsystem) {
    return Commands.sequence(new IntakeCommand(launcherSubsystem, lightsSubsystem),new WaitCommand(0.3),
        new ShootCommand(launcherSubsystem, lightsSubsystem), 
        new DriveBackwardsDistance(subsystem, 40));
  }

  public static Command ShootAuto(LauncherSubsystem launcherSubsystem,
      LightsSubsystem lightsSubsystem) {
    return Commands.sequence(new ShootCommand(launcherSubsystem, lightsSubsystem));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
