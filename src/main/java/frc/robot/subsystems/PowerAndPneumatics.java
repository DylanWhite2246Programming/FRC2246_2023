// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.Ports;

public class PowerAndPneumatics extends SubsystemBase {
  private static PowerDistribution pdh;
  private static PneumaticHub ph;
  private static Compressor compressor;

  /** Creates a new PowerAndPneumatics. */
  public PowerAndPneumatics() {
    pdh = new PowerDistribution(Ports.kPDHCANID, ModuleType.kRev);
    ph = new PneumaticHub(Ports.kPHCANID);
    compressor = ph.makeCompressor();

  }

  /**
   * @param port the port to querry
   * @return the amp draw of a given port
   */
  public double getPortAmperage(int port){return pdh.getCurrent(port);}
  /**
   * @return the voltage of the battery
   */
  public double getBatteryVoltage(){return pdh.getVoltage();}
  /**
   * @return the amp draw of the entire robot
   */
  public double getAmpDraw(){return pdh.getTotalCurrent();}
  /**
   * @return the amount of power being consumed from the entire robot
   */
  public double getPowerConsumtion(){return pdh.getTotalPower();}
  /**
   * @return the amount of energy consumed since last reset
   */
  public double getEnergyConsumtion(){return pdh.getTotalEnergy();}
  /**
   * @return tempreture of the pdh board
   */
  public double getPDHTemprature(){return pdh.getTemperature();}
  /**
   * @param value value to set the switchable channel true is on
   */
  public void setSwitchableChannel(boolean value){pdh.setSwitchableChannel(value);}
  /**gets status of switchable channel true is on */
  public boolean getSwitchableChannel(){return pdh.getSwitchableChannel();}


  /**
   * @return preasure of the pnuematic system
   */
  public double getPreasure(){return compressor.getPressure();}//TODO Check this
  /**turns on compressor with predefined limits in Robot.Constants file */
  public void turnOnCompressor(){compressor.enableAnalog(Constants.pressure[0], Constants.pressure[1]);}
  /**turns off compressor */
  public void turnOffCompressor(){compressor.disable();}
  /**turns on compressor with predefined limits in Robot.Constants file */
  public CommandBase turnOnCompressorCommand(){return runOnce(()->turnOnCompressor());}
  /**turns off compressor */
  public CommandBase turnOffCompressorCommand(){return runOnce(()->turnOffCompressor());}
  /**turns on targeting lights */
  public CommandBase turnOnHeadlights(){return runOnce(()->setSwitchableChannel(true));}
  /**turns off targeting lights */
  public CommandBase turnOffHeadlights(){return runOnce(()->setSwitchableChannel(false));}
  /**flashes headlights twice ending on the initial value */
  public CommandBase flashHeadlights(){
    boolean initialvalue = getSwitchableChannel();
    return new SequentialCommandGroup(
      turnOnHeadlights(),
      new WaitCommand(.2),
      turnOffHeadlights(),
      new WaitCommand(.2),
      turnOnHeadlights(),
      new WaitCommand(.2),
      turnOffHeadlights(),
      new WaitCommand(.2),
      initialvalue?turnOnHeadlights():null
    );
  }
  /**
   * @return the active satus of the compressor true means air is being compressed
   */
  public boolean getCompressorActive(){return compressor.isEnabled();}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
