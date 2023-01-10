// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.Ports;
import frc.robot.Constants.RobotConstruction;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class Arm extends ProfiledPIDSubsystem {
  private static CANSparkMax m1, m2;
  private static DoubleSolenoid claw = new DoubleSolenoid(
    PneumaticsModuleType.REVPH, 
    Ports.kClawForwardPort, 
    Ports.kClawReversePort
  );
  private static DoubleSolenoid extention = new DoubleSolenoid(
    PneumaticsModuleType.REVPH, 
    Ports.kExtentionForwardPort,
    Ports.kExtentionReversePort
  );
  private static DoubleSolenoid stopper = new DoubleSolenoid(
    PneumaticsModuleType.REVPH, 
    Ports.kStopperForwardPort, 
    Ports.kStopperReversePort
  );
  private static DigitalInput lowerLimit, upperLimit;
  private static DutyCycleEncoder encoder;
  private static final double extendedKSdelta=0;
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
  public boolean atGoal(){return getController().atGoal();}

  
  public CommandBase openClaw(){return runOnce(()->claw.set(Value.kForward));}
  public CommandBase closeClaw(){return runOnce(()->claw.set(Value.kReverse));}
  public CommandBase extendArm(){return runOnce(()->extention.set(Value.kForward));}
  public CommandBase retractArm(){return runOnce(()->extention.set(Value.kReverse));}
  public CommandBase extendStopper(){return runOnce(()->stopper.set(Value.kForward));}
  public CommandBase retractStopper(){return runOnce(()->stopper.set(Value.kReverse));}

  public CommandBase setArmPosistion(double x){
    return runOnce(
      ()->{
        setGoal(new State(x, 0));
        enable();
      }
    );
  }
  
  public CommandBase posistion0(){
    return Commands.sequence(
      retractArm(),
      retractStopper(),
      setArmPosistion(RobotConstruction.kArmEncoderOffset),
      openClaw()
      );
    }
    public CommandBase posistion1(){return Commands.sequence(setArmPosistion(0),extendArm());}
    public CommandBase posistion2(){return Commands.sequence(retractArm(), setArmPosistion(0));}
    public CommandBase posistion3(){return Commands.sequence(setArmPosistion(0),extendArm());}

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
      if(extention.get()==Value.kForward){
        m1.setVoltage(output+feedForward.calculate(setpoint.position, setpoint.velocity)+extendedKSdelta);
      }
      else{m1.setVoltage(output+feedForward.calculate(setpoint.position, setpoint.velocity));}
      //add the calculated feedforward to the pid output to get the motor output
      
    }
  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return encoder.getAbsolutePosition()+RobotConstruction.kArmEncoderOffset;
  }

  @Override
  public void periodic(){
    super.periodic();
  }
}
