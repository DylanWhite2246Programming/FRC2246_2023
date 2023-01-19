// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.RobotConstruction;

public class Drivetrain extends SubsystemBase {
  private static CANSparkMax l1,l2,r1,r2;
  private static DifferentialDrive drive;
  private static RelativeEncoder lEncoder, rEncoder;
  private static AHRS navx = new AHRS();  
  private static DoubleSolenoid brakeSolenoid;
  private Trigger brakeTrigger = new Trigger(this::getBrake);

  private static DifferentialDriveKinematics kinematics;
  private static DifferentialDriveOdometry odometry;
  private DifferentialDrivePoseEstimator drivePoseEstimator;

  private static PIDController turnController;

  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {
    //define variables
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
    brakeSolenoid.set(Value.kReverse);
    brakeTrigger
      .onTrue(runOnce(()->drive.setMaxOutput(0)))
      .onFalse(setNormalSpeed());


    drive = new DifferentialDrive(l1,r1);

    lEncoder=l1.getEncoder();
    lEncoder.setPositionConversionFactor(RobotConstruction.kPositionConversionFactor);
    lEncoder.setVelocityConversionFactor(RobotConstruction.kVelocityConversionFactor);

    rEncoder=r1.getEncoder();
    rEncoder.setPositionConversionFactor(RobotConstruction.kPositionConversionFactor);
    rEncoder.setVelocityConversionFactor(RobotConstruction.kVelocityConversionFactor);


    kinematics = new DifferentialDriveKinematics(RobotConstruction.kTrackWidth);
    odometry = new DifferentialDriveOdometry(
      getRotation2d(), getLeftDistance(), getRightDistance()
    );
    drivePoseEstimator = new DifferentialDrivePoseEstimator(
      kinematics, 
      getRotation2d(), 
      getLeftDistance(),
      getRightDistance(), 
      getPose2d()
    );
    turnController = new PIDController(.5, 0, 0);
      
    //zero yaw for beginning of match
    navx.zeroYaw();
    l2.follow(l1); r2.follow(r1);
    //invert left side
    l1.setInverted(true);
  }

  /**
   * @return returns the differential drive wheel speed of the robot 
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(lEncoder.getVelocity(), rEncoder.getVelocity());
  }
  /**
   * @return converts wheel speed to chassis speed and returns chassis speed
   */
  public ChassisSpeeds getChassisSpeed(){return kinematics.toChassisSpeeds(getWheelSpeeds());}

  /**
   * directly runs drivetrain with voltage parameters
   * @param lv left voltage
   * @param rv right voltage
   */
  public void driveVolts(double lv, double rv){
    l1.setVoltage(lv); r1.setVoltage(rv); 
  }

  /**@return turn rate of the robot in radians per second */
  public double getTurnRate(){return navx.getRate();}
  /**@return yaw in radians */
  public double getYaw(){return navx.getAngle();}
  public Rotation2d getRotation2d(){return navx.getRotation2d();}
  /**@return pitch in degrees*/
  public double getPitch(){return navx.getPitch();}

  /**@return distance in meters*/
  public double getLeftDistance(){return lEncoder.getPosition();}
  /**@return distance in meters*/
  public double getRightDistance(){return rEncoder.getPosition();}
  /**@return velocity in meters per second*/
  public double getLeftVelocity(){return lEncoder.getVelocity();}
  /**@return velocity in meters per second*/
  public double getRightVelocity(){return rEncoder.getVelocity();}

  /**@return returns pose in meters*/
  public Pose2d getPose2d(){return odometry.getPoseMeters();} 
  public DifferentialDriveKinematics getKinematics(){return kinematics;}

  public CommandBase STOP(){return runOnce(()->drive.stopMotor());}
  public CommandBase engageBrake(){return STOP().andThen(runOnce(()->brakeSolenoid.set(Value.kForward)));}
  public CommandBase disengageBrake(){return runOnce(()->brakeSolenoid.set(Value.kReverse));}
  /**@return true when brake engaged */
  public boolean getBrake(){return brakeSolenoid.get()==Value.kForward;}

  public CommandBase operatorDrive(DoubleSupplier x, DoubleSupplier z){
    return run(
      ()->{
        //if(Math.abs(z.getAsDouble())<OperatorConstants.kDriveStraightThreashold){
        //  drive.arcadeDrive(z.getAsDouble(), turnController.calculate(getChassisSpeed().omegaRadiansPerSecond, 0));
        //}else{
        //  drive.arcadeDrive(x.getAsDouble(), z.getAsDouble());
        //}
        drive.arcadeDrive(x.getAsDouble(), z.getAsDouble());
      }
    );
  }

  public CommandBase setFullSpeed(){return runOnce(()->drive.setMaxOutput(1));}
  public CommandBase setNormalSpeed(){return runOnce(()->drive.setMaxOutput(OperatorConstants.kNormalSpeed));}
  public CommandBase setSlowSpeed(){return runOnce(()->drive.setMaxOutput(OperatorConstants.kSlowSpeed));}

  public CommandBase setIdleMode(IdleMode mode){return runOnce(()->{
    r1.setIdleMode(mode);
    r2.setIdleMode(mode);
    l1.setIdleMode(mode);
    l2.setIdleMode(mode);
  });}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //update odometry
    odometry.update(getRotation2d(), getLeftDistance(), getRightDistance());
    drivePoseEstimator.update(getRotation2d(), getLeftVelocity(), getLeftDistance());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
