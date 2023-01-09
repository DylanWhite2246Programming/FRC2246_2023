// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Ports;

public class PowerAndPneumatics extends SubsystemBase {
  private static PowerDistribution pdh;
  private static PneumaticHub ph;
  /** Creates a new PowerAndPneumatics. */
  public PowerAndPneumatics() {
    pdh = new PowerDistribution(Ports.kPDHCANID, ModuleType.kRev);
    ph = new PneumaticHub(Ports.kPHCANID);
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


  /**
   * @return preasure of the pnuematic system
   */
  public double getPreasure(){return ph.getPressure(0);}//TODO Check this
  /**turns on compressor with predefined limits in Robot.Constants file */
  public void turnOnCompressor(){ph.enableCompressorAnalog(Constants.pressure[0], Constants.pressure[1]);}
  /**turns off compressor */
  public void turnOffCompressor(){ph.disableCompressor();}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
