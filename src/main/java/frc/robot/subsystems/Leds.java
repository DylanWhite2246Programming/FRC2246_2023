// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class Leds extends SubsystemBase {
  private static AddressableLED leds;
  private static AddressableLEDBuffer ledBuffer;
  /** Creates a new Leds. */
  public Leds() {
    leds=new AddressableLED(Ports.kLedPort);
    ledBuffer = new AddressableLEDBuffer(60);//TODO change to led length
    leds.setLength(ledBuffer.getLength());
    leds.setData(ledBuffer);
    leds.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
