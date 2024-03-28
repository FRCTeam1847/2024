// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
   private final WPI_TalonSRX m_motor = new WPI_TalonSRX(2);
  /** Creates a new Climber. */
  public ClimberSubsystem() {
    m_motor.setInverted(false);
  }

  public void liftMotor (){
    m_motor.set(-1);
  }
  public void stopMotor(){
    m_motor.stopMotor();
  }

/**
 * Runs climber up at full speed
 * @return
 */
  public Command Climb(){
   return run(
    () -> {
      m_motor.set(1);
    })
    .finallyDo(interrupted -> m_motor.stopMotor());
  }
  /**
   * Runs -1 on climber
   * @return
   */
    public Command Lower(){
   return run(
    () -> {
      m_motor.set(-1);
    })
    .finallyDo(interrupted -> m_motor.stopMotor());
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
