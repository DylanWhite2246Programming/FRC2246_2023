// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  static boolean isBlueAlliance(){return DriverStation.getAlliance()==Alliance.Blue;}
  public static boolean autonSuccessful = false;
  public static final int[] pressure = new int[]{75,90};
  public static class Ports{
    //CANID
    public static final int kL1CANID = 1;
    public static final int kL2CANID = 2;
    public static final int kR1CANID = 3;
    public static final int kR2CANID = 4;
    public static final int kArmMotor1Port = 5;
    public static final int kArmMotor2Port = 6;
    public static final int kPDHCANID = 7;
    public static final int kPHCANID = 8;
    //PWN
    public static final int kLedPort = 0;
    //DIO
    public static final int kArmEncoderPort = 0;
    public static final int kArmLowerLimitPort = 1;
    public static final int kArmUpperLimitPort = 2;
    public static final int kBoomLimitPort = 3;
    //SOLENOIDS
    public static final int kBrakeForwardPort = 0;
    public static final int kBrakeReversePort = 1;    
    public static final int kCubeClawForwardPort = 2;
    public static final int kConeClawForwardPort = 3;
    public static final int kCubeClawReversePort = 4;
    public static final int kConeClawReversePort = 5;
    public static final int kExtentionForwardPort = 6;
    public static final int kExtentionReversePort = 7;
  }
  public static class AutonConstants{
    public static final PIDController kLeftController = 
      new PIDController(0, 0, 0);
    public static final PIDController kRightController =
      new PIDController(0, 0, 0);
    public static final SimpleMotorFeedforward kFeedForward = 
      new SimpleMotorFeedforward(0, 0, 0);
    public static final RamseteController kRamseteController = 
      new RamseteController(0, 0);
    public static final double kDriveXKp = 
      (kLeftController.getP()+kRightController.getP())/2;
  } 
  public static class AutonPaths{
    public static Trajectory getTaxiPassScale(){
      Trajectory x = new Trajectory();
      try{
        x = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("taxipassscaleblue.wpilib.json"));
      }catch(IOException ex){
        DriverStation.reportError("Unable to open trajectory: taxipassscaleblue.wpilib.json", ex.getStackTrace());
      }
      if(!isBlueAlliance()){x.relativeTo(new Pose2d(16.534, 8.005, new Rotation2d(Math.PI)));}
      return x;
    }
    public static Trajectory getBackOnToScale(){
      Trajectory x = new Trajectory();
      try{
        x = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("backontoscaleblue.wpilib.json"));
      }catch(IOException ex){
        DriverStation.reportError("Unable to open trajectory: backontoscaleblue.wpilib.json", ex.getStackTrace());
      }
      if(!isBlueAlliance()){x.relativeTo(new Pose2d(16.534, 8.005, new Rotation2d(Math.PI)));}
      return x;
    }
  }
  public static class FieldConstants{
    public static double feildLength=16.534, feildWidth=8.005;
    public static Translation2d getGP0(){return new Translation2d(
        isBlueAlliance()?7.07:feildLength-7.07, .92
      );
    }
    public static Translation2d getGP1(){return new Translation2d(
        isBlueAlliance()?7.07:feildLength-7.07, 2.14
      );
    }
    public static Translation2d getGP2(){return new Translation2d(
        isBlueAlliance()?7.07:feildLength-7.07, 3.36
      );
    }
    public static Translation2d getGP3(){return new Translation2d(
        isBlueAlliance()?7.07:feildLength-7.07, 4.59
      );
    }
    private static final double[] pegRowXArray 
      = new double[]{.669,.236474};
    private static final double[] pegColumnYArray 
      = new double[]{.508,1.626,2.235,3.353,3.912,5.029};
    /**
     * @param row the row the peg is in 0 = low; 1 = high
     * @param column the column the peg is in 0 is closest to the wall
     * @return the translation of the peg using the global position origin
     */
    public static Pose2d getPeg(int row, int column){
      double x = pegRowXArray[row];
      Rotation2d rot = new Rotation2d();
      //if red alliance change the pegs to the other side of the field
      if(!isBlueAlliance()){x=feildLength-x;rot=Rotation2d.fromDegrees(180);}
      return new Pose2d(x, pegColumnYArray[column], rot);
    }
    public static Pose2d choosePeg(int row, Pose2d robotPose){
      int choosenPeg=0; double shortestDistance=Integer.MAX_VALUE;
      for(int i=0;i<=5;i++){
        if(robotPose.getTranslation().getDistance(getPeg(row, i).getTranslation())<shortestDistance){
          shortestDistance=robotPose.getTranslation().getDistance(getPeg(row, i).getTranslation());
        }
      }
      return getPeg(row, choosenPeg);
    }
    private static final double[] shelfRowXArray
      = new double[]{.286,.505};
    private static final double[] shelfColumnYArray
      = new double[]{1.07,2.743,4.42};
    public static Pose2d getShelf(int row, int column){
      double x = shelfRowXArray[row];
      Rotation2d rot= new Rotation2d();
      //if red alliance change the pegs to the other side of the field
      if(!isBlueAlliance()){x=feildLength-x;rot=Rotation2d.fromDegrees(180);}
      return new Pose2d(x, shelfColumnYArray[column], rot);
    }
    public static Pose2d chooseShelf(int row, Pose2d robotPose){
      int choosenShelf=0; double shortestDistance=-1;
      for(int i=0;i<=2;i++){
        if(robotPose.getTranslation().getDistance(getShelf(row, i).getTranslation())<shortestDistance||shortestDistance==-1){
          shortestDistance=robotPose.getTranslation().getDistance(getShelf(row, i).getTranslation());
        }
      }
      return getShelf(row, choosenShelf);
    }
  }
  public static class OperatorConstants {
    /*percent */
    public static final double kDriveStraightThreashold = .0075;
    
    public static double tune(double x){
      return Math.signum(x)*x*x;
    }
    public static final double kRobotTopSpeed = 10;
    public static final double kSlowSpeed = .3*kRobotTopSpeed, kNormalSpeed = .7*kRobotTopSpeed;
    public static final double normalRotateSpeed = 1, kSlowRotateSpeed = .7;
  }
  public static class RobotConstruction{
    /*meters */
    public static final double kTrackWidth = Units.inchesToMeters(0);
    /**meters */ //10.71 is gearing ratio //wheel diameter is 6
    public static final double kPositionConversionFactor = 1/(10.71*Math.PI*Units.inchesToMeters(6));
    /**meters per second */
    public static final double kVelocityConversionFactor = kPositionConversionFactor/60;
    /*radians */
    public static final double kArmEncoderOffset = 0;
    /**radians */
    public static final double kCollisionAngle = 0;
    public static final double 
      backTopPosition = 0,
      backMidPosistion  = 0,
      backLowPosition = 0,
      intakePostion = 0,
      frontLowPosition = 0,
      frontMidllePosition = 0;
    //robot to center intake claw
    public static final Transform2d kRobotToIntake =
      new Transform2d(new Translation2d(0, 0), new Rotation2d());  
    public static final Transform3d kRobotToCam =
      new Transform3d(
        new Translation3d(0, 0, 0), 
        new Rotation3d(0, 0, 0)
    );
  }
}
