// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.Ports;
import frc.robot.Constants.RobotConstruction;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

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
  private static DigitalInput lowerLimit, upperLimit, boomLimit;
  private static DutyCycleEncoder encoder;
  private static final double extendedKSdelta = 0;
  private static final ArmFeedforward feedForward 
    = new ArmFeedforward(0, 0, 0, 0);//TODO set values
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

  /**way of overiding move arm */
  public CommandBase setGoalCommand(double goal){return runOnce(()->{setGoal(goal);enable();});}

  private CommandBase moveArm(double value){
    return new ConditionalCommand(
      //retract arm and wait for it to reach limit
      new SequentialCommandGroup(
        retractArm(),
        new WaitUntilCommand(this::getBoomLimit).withTimeout(3),
        setGoalCommand(value)
      ),
      //if collision will not happen move arm
      setGoalCommand(value), 
      //when the goal and curent position are on differnt sides of the robot the arm must be retracted
      ()->(Math.signum(value)!=Math.signum(this.getMeasurement()))||value==0
    ).until(this::atGoal);
  }

  public CommandBase moveToBackTopPosition(){return moveArm(RobotConstruction.backTopPosition).andThen(extendArm());}
  public CommandBase moveToBackMiddlePostion(){return moveArm(RobotConstruction.backMidPosistion).andThen(extendArm());}
  public CommandBase moveToBackLowPosition(){return moveArm(RobotConstruction.backLowPosition).andThen(extendArm());}
  public CommandBase moveToZeroPosition(){return moveArm(0).andThen(()->disable());}
  public CommandBase moveToIntakePosition(){return moveArm(RobotConstruction.intakePostion).andThen(extendArm());}
  public CommandBase moveToFrontGroudPosition(){return moveArm(RobotConstruction.frontLowPosition).andThen(extendArm());}
  public CommandBase moveToFrontMiddlePosition(){return moveArm(RobotConstruction.frontMidllePosition).andThen(extendArm());}



  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    if(getBoomLimit()){
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
