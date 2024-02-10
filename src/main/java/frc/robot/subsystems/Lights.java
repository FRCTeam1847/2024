// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
    AddressableLED m_led = new AddressableLED(9);
  AddressableLEDBuffer m_ledBuffer ;
  /** Creates a new Lights. */
  public Lights() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
