// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class LiftClimberCommand extends Command {
  private ClimberSubsystem climberSubsystem;
    private Timer localTimer = new Timer();
    double timeOut = 8;

  /** Creates a new LiftClimberCommand. */
  public LiftClimberCommand(ClimberSubsystem _climberSubsystem) {
    climberSubsystem =_climberSubsystem;
    addRequirements(climberSubsystem) ;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    localTimer.reset();
    localTimer.start();
    climberSubsystem.liftMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return localTimer.get() > timeOut;
  }
}
