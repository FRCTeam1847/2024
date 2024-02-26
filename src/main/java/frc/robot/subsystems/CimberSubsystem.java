// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CimberSubsystem extends SubsystemBase {
   private final WPI_TalonSRX m_motor = new WPI_TalonSRX(2);
  /** Creates a new Cimber. */
  public CimberSubsystem() {
    m_motor.setInverted(false);
  }

  public Command Climb(){
   return run(
    () -> {
      m_motor.set(1);
    })
    // Drive forward at specified speed
    // Stop the drive when the command ends
    .finallyDo(interrupted -> m_motor.stopMotor());
  }
    public Command Lower(){
   return run(
    () -> {
      m_motor.set(-1);
    })
    // Drive forward at specified speed
    // Stop the drive when the command ends
    .finallyDo(interrupted -> m_motor.stopMotor());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
