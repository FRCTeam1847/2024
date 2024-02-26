// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
  // private TalonFX m_motor = new TalonFX(6, "rio");
    private final CANSparkMax m_motor = new CANSparkMax(5, MotorType.kBrushless);

  /** Creates a new Feeder. */
  public FeederSubsystem() {
  }

  public void IntakeNote() {
    m_motor.set(-0.25);
  }

  public void ShootNote() {
    m_motor.set(0.8);
  }

  public void Stop() {
    m_motor.stopMotor();
  }

  public Command StopCommand() {
    return new Command() {
      @Override
      public void execute() {
        Stop();
      }
    };
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
