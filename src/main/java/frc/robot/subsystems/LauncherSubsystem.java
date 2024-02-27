// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase {
  private final WPI_TalonSRX m_launch = new WPI_TalonSRX(1);
  private final CANSparkMax m_feed = new CANSparkMax(5, MotorType.kBrushless);

  private final double intakeSpeed = -0.5;
  private final double launchSpeed = 1;
  private final double feedSpeed = 0.25;
  /** Creates a new Launcher. */
  public LauncherSubsystem() {
    m_launch.setInverted(true);
  }

  public void StopMotors() {
    m_launch.stopMotor();
    m_feed.stopMotor();
  }

  private boolean getBottomSwitchValue() {
    return m_launch.isRevLimitSwitchClosed() == 0;
  }

  private boolean getTopSwitchValue() {
    return m_launch.isFwdLimitSwitchClosed() == 0;
  }

  public void setLaunchWheel(double speed) {
    m_launch.set(speed);
  }

  public void setFeedWheel(double speed) {
    m_feed.set(speed);
  }

  public Command intakeCommand() {
    return run(
        () -> {
          m_launch.set(intakeSpeed);
          m_feed.set(-feedSpeed);
        })
        // .onlyIf(() -> getTopSwitchValue())
        .until(
            () -> getBottomSwitchValue())
        // .withTimeout(2)
        .finallyDo(interrupted -> StopMotors());
  }

  public Command prepareLaunch(){
    return run(()->m_launch.set(launchSpeed));
  }

  public Command launchNoteCommand(){
    return run(()-> {
      m_launch.set(launchSpeed);
      m_feed.set(feedSpeed);
    });
  }





  public BooleanSupplier TopSwitchPressed() {
    // Query some boolean state, such as a digital sensor.
    return new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        return getTopSwitchValue();
      }
    };

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Launch Wheel", m_launch.getMotorOutputVoltage());
    SmartDashboard.putNumber("Feed Wheel", m_feed.getEncoder().getVelocity());
  }
}
