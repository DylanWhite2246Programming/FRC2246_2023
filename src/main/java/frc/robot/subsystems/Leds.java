// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Ports;

public class Leds extends SubsystemBase {
  private static AddressableLED leds;
  private static AddressableLEDBuffer ledBuffer;
  /** Creates a new Leds. */
  public Leds() {
    leds = new AddressableLED(Ports.kLedPort);
    ledBuffer = new AddressableLEDBuffer(60);//TODO change to led length
    leds.setLength(ledBuffer.getLength());
    leds.setData(ledBuffer);
    leds.start();
  }

  /**sets all leds in the strip to the color provided */
  public static void setColor(Color color){
    for(int i=0;i<ledBuffer.getLength();i++){
      ledBuffer.setLED(i, color);
    }
    leds.setData(ledBuffer);
  }
  /**toggles the color of the led strip(s) from yellow and purple 
   * with yellow being the default color
   */
  public CommandBase toggleColor(){
    return runOnce(()->{
        if(ledBuffer.getLED(0)==Color.kYellow){setColor(Color.kPurple);}
        else{setColor(Color.kYellow);}
      }
    );
  }
  public CommandBase flashLeds(){
    return new SequentialCommandGroup(
      turnOnLeds(),
      new WaitCommand(.2),
      turnOffLeds(),
      new WaitCommand(.2),
      turnOnLeds(),
      new WaitCommand(.2),
      turnOffLeds(),
      new WaitCommand(.2),
      turnOnLeds()
    );
  }
  public CommandBase turnOnLeds(){return runOnce(()->leds.start());}
  public CommandBase turnOffLeds(){return runOnce(()->leds.stop());}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
