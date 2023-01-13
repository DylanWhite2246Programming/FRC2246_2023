// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static boolean autonSuccessful = false;
  public static final int[] pressure = new int[]{75,90};
  public static class Ports{
    //CANID
    public static final int kPDHCANID = 0;
    public static final int kL1CANID = 1;
    public static final int kL2CANID = 2;
    public static final int kR1CANID = 3;
    public static final int kR2CANID = 4;
    public static final int kArmMotor1Port = 5;
    public static final int kArmMotor2Port = 6;
    public static final int kPHCANID = 7;
    //SOLENOIDS
    public static final int kBrakeForwardPort = 0;
    public static final int kBrakeReversePort = 1;
    public static final int kClawForwardPort = 2;
    public static final int kClawReversePort = 3;
    public static final int kExtentionForwardPort = 4;
    public static final int kExtentionReversePort = 5;
    //DIO
    public static final int kArmEncoderPort = 0;
    public static final int kArmLowerLimitPort = 1;
    public static final int kArmUpperLimitPort = 2;
    public static final int kBoomLimitPort = 3;
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
  public static class FieldConstants{
    //public static final Translation2d 
  }
  public static class OperatorConstants {
    /*percent */
    public static final double kDriveStraightThreashold = .0075;
    public static final int kDriverControllerPort = 0;
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
    public static final Transform3d kRobotToCam =
      new Transform3d(
        new Translation3d(0, 0, 0), 
        new Rotation3d(0, 0, 0)
      );
  }
}
