// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LauncherSubsystem extends SubsystemBase {
  
  private final DoubleSolenoid m_launchMode = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
  public final WPI_TalonSRX m_launch = new WPI_TalonSRX(1);
  public final CANSparkMax m_feed = new CANSparkMax(5, MotorType.kBrushless);
  public final DigitalInput noteSwitch = new DigitalInput(4);

  boolean engaged = false;

  /** Creates a new Launcher. */
  public LauncherSubsystem() {
    m_launch.setInverted(true);
    m_launch.setNeutralMode(NeutralMode.Brake);
    m_feed.setIdleMode(IdleMode.kBrake);
  }

  public void StopMotors() {
    engaged = false;
    m_launch.stopMotor();
    m_feed.stopMotor();
  }

  // public boolean getBottomSwitchValue() {
  //   return m_launch.isRevLimitSwitchClosed() == 0;
  // }

  public boolean getTopSwitchValue() {
    return m_launch.isFwdLimitSwitchClosed() == 0;
  }

  public void setLaunchWheel(double speed) {
    engaged = true;
    m_launch.set(speed);
  }

  public void setFeedWheel(double speed) {
    engaged = true;
    m_feed.set(speed);
  }

  public void enableLaunchMode() {
    m_launchMode.set(DoubleSolenoid.Value.kForward);
  }

  public void disableLaunchMode() {
    m_launchMode.set(DoubleSolenoid.Value.kReverse);
  }

  public final Trigger hasTopNote = new Trigger(() -> !engaged && getTopSwitchValue());

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Launch Wheel", m_launch.getMotorOutputVoltage());
    SmartDashboard.putNumber("Feed Wheel", m_feed.getEncoder().getVelocity());
  }
}
