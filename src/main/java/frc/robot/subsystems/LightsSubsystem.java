// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsSubsystem extends SubsystemBase {
  AddressableLED m_led = new AddressableLED(0);
  public AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(150);

  public int LeftLights = 20;
  public int TopLights = 32;
  public int RightLights = 55;

  public Color offColor = new Color(0, 0, 0);
  public Color greenColor = new Color(0, 255, 0);
  public Color redColor = new Color(255, 0, 0);
  public Color blueColor = new Color(0, 0, 255);
  public Color whiteColor = new Color(255, 255, 255);
  public Color goldColor = new Color(255, 215, 0);

  /** Creates a new Lights. */
  public LightsSubsystem() {
    m_led.setLength(m_ledBuffer.getLength());

    StartUp();

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
    System.out.println("Starting LEDS");
  }

  public void StartUp() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, goldColor);
    }
  }

  public void UpdateBaseOnAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (Alliance.Blue == alliance.get()) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Set alliance color to blue
          m_ledBuffer.setLED(i, blueColor);
        }
      } else {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Set alliance color to red
          m_ledBuffer.setLED(i, redColor);
        }
      }
    }
  }

  public void LightsOff() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, offColor);
    }
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      if (DriverStation.isDSAttached()) {
        UpdateBaseOnAlliance();
      }

    } else {
      if (DriverStation.isAutonomous()) {
        LightsOff();
      }
    }
    m_led.setData(m_ledBuffer);
  }
}
