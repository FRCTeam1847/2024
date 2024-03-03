// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class LauncherSubsystem extends SubsystemBase {
  public final WPI_TalonSRX m_launch = new WPI_TalonSRX(1);
  public final CANSparkMax m_feed = new CANSparkMax(5, MotorType.kBrushless);

  /** Creates a new Launcher. */
  public LauncherSubsystem() {
    m_launch.setInverted(true);
  }

  public void StopMotors() {
    m_launch.stopMotor();
    m_feed.stopMotor();
  }

  public boolean getBottomSwitchValue() {
    return m_launch.isRevLimitSwitchClosed() == 0;
  }

  public boolean getTopSwitchValue() {
    return m_launch.isFwdLimitSwitchClosed() == 0;
  }

  public void setLaunchWheel(double speed) {
    m_launch.set(speed);
  }

  public void setFeedWheel(double speed) {
    m_feed.set(speed);
  }

  public final Trigger hasTopNote = new Trigger(()->getTopSwitchValue());


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Launch Wheel", m_launch.getMotorOutputVoltage());
    SmartDashboard.putNumber("Feed Wheel", m_feed.getEncoder().getVelocity());
  }
}
