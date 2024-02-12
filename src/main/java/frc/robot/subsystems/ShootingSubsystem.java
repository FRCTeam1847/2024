// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShootingSubsystem extends SubsystemBase {
  private final WPI_TalonSRX m_motor = new WPI_TalonSRX(5);

  /** Creates a new IntakeSubsystem. */
  public ShootingSubsystem() {

  }

  private boolean getSwitchValue() {
    return m_motor.isRevLimitSwitchClosed() == 0;
  }

  public void IntakeNote() {
    m_motor.set(ControlMode.PercentOutput, -0.2);
  }

  public void ShootNote() {
    m_motor.set(ControlMode.PercentOutput, 0.25);
  }

  public void Stop() {
    m_motor.stopMotor();
  }

  public BooleanSupplier NoteSwitchPressed() {
    // Query some boolean state, such as a digital sensor.
    return new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        return getSwitchValue();
      }
    };

  }

  public BooleanSupplier NoteSwitchNotPressed() {
    // Query some boolean state, such as a digital sensor.
    return new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        return !getSwitchValue();
      }
    };

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
