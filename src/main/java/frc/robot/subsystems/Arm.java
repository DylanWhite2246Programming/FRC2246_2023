// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.Ports;
import frc.robot.Constants.RobotConstruction;
import frc.robot.commands.MoveArm;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  private static DigitalInput lowerLimit, upperLimit,boomLimit;
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
    boomLimit = new DigitalInput(Ports.kBoomLimitPort);
    encoder = new DutyCycleEncoder(Ports.kArmEncoderPort);

    //configured variables
      //set distance per pulse as tau radians
    encoder.setDistancePerRotation(2*Math.PI);
    m2.follow(m1);
  }

  /** @return state of lower limit true = pressed*/
  public boolean getLowerLimit(){return lowerLimit.get();}
  /** @return state of upper limit true = pressed*/
  public boolean getUpperLimit(){return upperLimit.get();}
  /** @return state of the boom limit */
  public boolean getBoomLimit(){return boomLimit.get();}
  public boolean atGoal(){return getController().atGoal();}

  
  public CommandBase openClaw(){return runOnce(()->claw.set(Value.kForward));}
  public CommandBase closeClaw(){return runOnce(()->claw.set(Value.kReverse));}
  public CommandBase extendArm(){return runOnce(()->extention.set(Value.kForward));}
  public CommandBase retractArm(){return runOnce(()->extention.set(Value.kReverse));}

  public CommandBase setGoalCommand(double goal){return runOnce(()->setGoal(goal));}

  public CommandBase posistion0(){
    return Commands.sequence(
        retractArm(),
        new MoveArm(this, RobotConstruction.kArmEncoderOffset),
        openClaw()
      );
    }
    public CommandBase posistion1(){return Commands.sequence(new MoveArm(this, 0),extendArm());}
    public CommandBase posistion2(){return Commands.sequence(retractArm(), new MoveArm(this, 0));}
    public CommandBase posistion3(){return Commands.sequence(new MoveArm(this, 0),extendArm());}

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
