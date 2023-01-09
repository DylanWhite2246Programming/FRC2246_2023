// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.Ports;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class Arm extends ProfiledPIDSubsystem {
  private static CANSparkMax m1, m2;
  private static DoubleSolenoid claw = new DoubleSolenoid(
    PneumaticsModuleType.REVPH, 
    Ports.kClawForwardPort, 
    Ports.kClawReversePort
  );
  private static DigitalInput lowerLimit, upperLimit;
  private static DutyCycleEncoder encoder;
  private static final ArmFeedforward feedForward = new ArmFeedforward(0, 0, 0, 0);//TODO set values
  /** Creates a new Arm. */
  public Arm() {
    super(
      // The ProfiledPIDController used by the subsystem
      new ProfiledPIDController(//TODO set values
        0,
        0,
        0,
        // The motion profile constraints
        new TrapezoidProfile.Constraints(0, 0)));
    //Defined variables
    m1 = new CANSparkMax(Ports.kArmMotor1Port, MotorType.kBrushless);
    m2 = new CANSparkMax(Ports.kArmMotor2Port, MotorType.kBrushless);
    lowerLimit = new DigitalInput(Ports.kArmLowerLimitPort);
    upperLimit = new DigitalInput(Ports.kArmUpperLimitPort);
    encoder = new DutyCycleEncoder(Ports.kArmEncoderPort);

    //configured variables
      //set distance per pulse as tau radians
    encoder.setDistancePerRotation(2*Math.PI);
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


  /**close claw */
  public void closeClaw(){claw.set(Value.kReverse);}
  /**open claw */
  public void openClaw(){claw.set(Value.kForward);}


  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    //calculate feedforward from setpoint
    var calculatedFeed = feedForward.calculate(setpoint.position, setpoint.velocity);
    //add the feedforward to the pid output to get the motor output
    m1.setVoltage(output+calculatedFeed);
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