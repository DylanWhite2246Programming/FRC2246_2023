// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
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
    //DIO
    public static final int kArmEncoderPort = 0;
    public static final int kArmForwardLimitPort = 1;
    public static final int kArmReverseLimitPort = 2;
  }
  public static class OperatorConstants {
    public static final double kDriveStraightThreashold = .05;
    public static final int kDriverControllerPort = 0;
  }
  public static final class RobotConstruction{
    public static final double kTrackWidth = 0;
  }
}
