// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.RobotConstruction;

import org.photonvision.EstimatedRobotPose;

public class Drivetrain extends SubsystemBase {
  private static CANSparkMax l1,l2,r1,r2;
  private static RelativeEncoder lEncoder, rEncoder;
  private static AHRS navx = new AHRS();  
  //private static DoubleSolenoid brakeSolenoid;
  private static double maxOutput = OperatorConstants.kNormalSpeed;
  private static double rotateScalar = OperatorConstants.normalRotateSpeed;


  private static DifferentialDriveKinematics kinematics;
  private static DifferentialDriveOdometry odometry;
  private static DifferentialDrivePoseEstimator drivePoseEstimator;

  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {
    //define variables
    l1 = new CANSparkMax(Ports.kL1CANID, MotorType.kBrushless);
    l2 = new CANSparkMax(Ports.kL2CANID, MotorType.kBrushless);
    r1 = new CANSparkMax(Ports.kR1CANID, MotorType.kBrushless);
    r2 = new CANSparkMax(Ports.kR2CANID, MotorType.kBrushless);

    //brakeSolenoid = new DoubleSolenoid(
    //  Ports.kPHCANID, 
    //  PneumaticsModuleType.REVPH, 
    //  Ports.kBrakeForwardPort, 
    //  Ports.kBrakeReversePort
    //);
    //brakeSolenoid.set(Value.kReverse);
    //new Trigger(this::getBrake)
    //  .onTrue(runOnce(()->maxOutput=0))
    //  .onFalse(setNormalSpeed());

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
      
    //zero yaw for beginning of match
    navx.zeroYaw();
    l2.follow(l1); r2.follow(r1);
    //invert left side
    l1.setInverted(true);
    lEncoder.setInverted(true);
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
  public Rotation2d getRotation2d(){return navx.getRotation2d().times(-1);}
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
  public Pose2d getPose2d(){
    if(DriverStation.isAutonomous()){
      return odometry.getPoseMeters();
    }else return drivePoseEstimator.getEstimatedPosition();
  } 
  public DifferentialDriveKinematics getKinematics(){return kinematics;}

  public CommandBase STOP(){return runOnce(()->{l1.stopMotor();r1.stopMotor();});}
  //public CommandBase engageBrake(){return STOP().andThen(runOnce(()->brakeSolenoid.set(Value.kForward)));}
  //public CommandBase disengageBrake(){return runOnce(()->brakeSolenoid.set(Value.kReverse));}
  ///**@return true when brake engaged */
  //public boolean getBrake(){return brakeSolenoid.get()==Value.kForward;}

  /**imput should be from -1 to 1 */
  public CommandBase operatorDrive(DoubleSupplier x, DoubleSupplier z){
    return run(
      ()->{
        DifferentialDriveWheelSpeeds speeds = kinematics.toWheelSpeeds(new ChassisSpeeds(x.getAsDouble()*maxOutput, 0, z.getAsDouble()*rotateScalar));
        driveVolts(
          AutonConstants.kFeedForward.calculate(speeds.leftMetersPerSecond)+AutonConstants.kLeftController.calculate(getLeftVelocity(), speeds.leftMetersPerSecond), 
          AutonConstants.kFeedForward.calculate(speeds.rightMetersPerSecond)+AutonConstants.kRightController.calculate(getRightVelocity(), speeds.rightMetersPerSecond)
        );
      }
    );
  }

  /**imput should be from in the bounds of the robots top speed*/
  public CommandBase computerDrive(DoubleSupplier x, DoubleSupplier z){
    return run(
      ()->{
        var speeds = kinematics.toWheelSpeeds(new ChassisSpeeds(x.getAsDouble(), 0, z.getAsDouble()));
        speeds.desaturate(OperatorConstants.kRobotTopSpeed);
        driveVolts(
          AutonConstants.kFeedForward.calculate(speeds.leftMetersPerSecond)+AutonConstants.kLeftController.calculate(getLeftVelocity(), speeds.leftMetersPerSecond), 
          AutonConstants.kFeedForward.calculate(speeds.rightMetersPerSecond)+AutonConstants.kRightController.calculate(getRightVelocity(), speeds.rightMetersPerSecond)
        );
      }
    );
  }

  public CommandBase setFullSpeed(){return runOnce(()->{
    maxOutput = OperatorConstants.kRobotTopSpeed;
    rotateScalar = OperatorConstants.kSlowRotateSpeed;
  });}
  public CommandBase setNormalSpeed(){return runOnce(()->{
    maxOutput = OperatorConstants.kNormalSpeed;
    rotateScalar = OperatorConstants.normalRotateSpeed;
  });}
  public CommandBase setSlowSpeed(){return runOnce(()->{
    maxOutput=OperatorConstants.kSlowSpeed;
    rotateScalar = OperatorConstants.normalRotateSpeed;
  });}

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
  }

  public void updateOdometry(Vision vision) {
    drivePoseEstimator.update(
      getRotation2d(), getLeftDistance(), getRightDistance()
    );

    // Also apply vision measurements. We use 0.3 seconds in the past as an example
    // -- on
    // a real robot, this must be calculated based either on latency or timestamps.
    Optional<EstimatedRobotPose> result =
      vision.getEstimatedGlobalPose(drivePoseEstimator.getEstimatedPosition());

    if (result.isPresent()) {
      EstimatedRobotPose camPose = result.get();
      drivePoseEstimator.addVisionMeasurement(
        camPose.estimatedPose.toPose2d(), camPose.timestampSeconds
      );
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
