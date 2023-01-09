// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.Ports;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class Arm extends ProfiledPIDSubsystem {
  private static CANSparkMax m1, m2;
  private static DigitalInput lowerLimit, upperLimit;
  private static DutyCycleEncoder encoder;
  /** Creates a new Arm. */
  public Arm() {
    super(
      // The ProfiledPIDController used by the subsystem
      new ProfiledPIDController(
        0,
        0,
        0,
        // The motion profile constraints
        new TrapezoidProfile.Constraints(0, 0)));
    m1 = new CANSparkMax(Ports.kArmMotor1Port, MotorType.kBrushless);
    m2 = new CANSparkMax(Ports.kArmMotor2Port, MotorType.kBrushless);
    encoder = new DutyCycleEncoder(Ports.kArmEncoderPort);
    encoder.setDistancePerRotation(360);
    lowerLimit = new DigitalInput(Ports.kArmLowerLimitPort);
    upperLimit = new DigitalInput(Ports.kArmUpperLimitPort);
    //TODO figure out offset
    m2.follow(m1);
  }

  /**
   * @return state of lower limit true = pressed
   */
  public boolean getLowerLimit(){return lowerLimit.get();}
  /**
   * @return state of upper limit true = pressed
   */
  public boolean getUpperLimit(){return upperLimit.get();}


  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return encoder.getAbsolutePosition();
  }

  @Override
  public void periodic(){
    super.periodic();
  }
}
