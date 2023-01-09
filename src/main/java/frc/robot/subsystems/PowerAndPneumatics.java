// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  public double getBatteryVoltage(){return pdh.getVoltage();}
  public double getAmpDraw(){return pdh.getTotalCurrent();}
  public double getPowerConsumtion(){return pdh.getTotalPower();}
  public double getEnergyConsumtion(){return pdh.getTotalEnergy();}
  public double getPDHTemprature(){return pdh.getTemperature();}
  public void setSwitchableChannel(boolean value){pdh.setSwitchableChannel(value);}

  public double getPreasure(){return ph.getPressure(0);}//TODO Check this

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
