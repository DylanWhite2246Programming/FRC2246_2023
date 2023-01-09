// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.RobotConstruction;

public class Drivetrain extends SubsystemBase {
  private static CANSparkMax l1,l2,r1,r2;
  private static DoubleSolenoid brakeSolenoid;
  private static RelativeEncoder lEncoder, rEncoder;  
  private static DifferentialDrive drive;

  private static DifferentialDriveKinematics kinematics;
  private static DifferentialDriveOdometry odometry;

  private static PIDController turnController;

  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {
    l1 = new CANSparkMax(Ports.kL1CANID, MotorType.kBrushless);
    l2 = new CANSparkMax(Ports.kL2CANID, MotorType.kBrushless);
    r1 = new CANSparkMax(Ports.kR1CANID, MotorType.kBrushless);
    r2 = new CANSparkMax(Ports.kR2CANID, MotorType.kBrushless);
    brakeSolenoid = new DoubleSolenoid(
      Ports.kPHCANID, 
      PneumaticsModuleType.REVPH, 
      Ports.kBrakeForwardPort, 
      Ports.kBrakeReversePort
    );
    drive = new DifferentialDrive(l1,r1);
    l2.follow(l1); r2.follow(r1);
    lEncoder=l1.getEncoder();
    rEncoder=r1.getEncoder();
    kinematics = new DifferentialDriveKinematics(RobotConstruction.kTrackWidth);
    odometry = new DifferentialDriveOdometry(null, 0, 0);//TODO change
    turnController = new PIDController(.5, 0, 0);
  }

  /**
   * @return returns the differential drive wheel speed of the robot 
   */
  public DifferentialDriveWheelSpeeds getDifferentialDriveWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(lEncoder.getVelocity(), rEncoder.getVelocity());
  }
  /**
   * @return converts wheel speed to chassis speed and returns chassis speed
   */
  public ChassisSpeeds getChassisSpeed(){return kinematics.toChassisSpeeds(getDifferentialDriveWheelSpeeds());}

  /**
   * @param x forward speed + is forward
   * @param z rotation speed + is clockwise
   * applys pid control loop to staty straight when oporator is not turning
   */
  public void operatorDrive(double x, double z){
    if(Math.abs(z)<OperatorConstants.kDriveStraightThreashold){
      drive.arcadeDrive(x, turnController.calculate(getChassisSpeed().omegaRadiansPerSecond, 0));
    }else{
      drive.arcadeDrive(x, z);
    }
  }

  /**applies brake to wheel */
  public static void applyHandBrake(){brakeSolenoid.set(Value.kReverse);}
  /**disengages brake to wheel */
  public static void disengageHandBrake(){brakeSolenoid.set(Value.kForward);}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
