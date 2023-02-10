// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.AutonPaths;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  /** TODO
   * autos we will have to make 
   *  1 game piece collect another and stay out of the way
   *  3 game piece starting left side with cone
   *  3 game piece starting left side with cube
   *  3 game piece starting right side with cone
   *  3 game piece starting right side with cube
   *  2 game piece and level starting left side with cone
   *  2 game piece and level starting left side with cube
   *  2 game piece and level starting right side with cone
   *  2 game piece and level starting right side with cube
   *  1 game piece and level starting in the middle (same for both alliance kinda)
   */

  private static RamseteCommand ramseteCommandGenerator(Drivetrain drive, Trajectory trajectory){
    return new RamseteCommand(
      trajectory, 
      drive::getPose2d, 
      new RamseteController(0, 0), 
      AutonConstants.kFeedForward, 
      drive.getKinematics(), 
      drive::getWheelSpeeds, 
      AutonConstants.kLeftController, 
      AutonConstants.kRightController, 
      drive::driveVolts, 
      drive
    );
  }

  /** Example static factory for an autonomous command. */
  //public static CommandBase exampleAuto(Drivetrain subsystem) {
  //  return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  //}
  //public static CommandBase autoLevel(Drivetrain drivetrain){
  //  return Commands.sequence(d)
  //}

  public static CommandBase twoGamePiecesAndLevel(Drivetrain drivetrain, Arm arm){
    return new SequentialCommandGroup(
      arm.moveToBackTopPosition(),
      arm.openClaw(),
      ramseteCommandGenerator(drivetrain, DriverStation.getAlliance()==Alliance.Blue?new Trajectory():new Trajectory())//drive to next game piece
        .alongWith(arm.moveToIntakePosition()),
      arm.closeClaw(),
      new WaitCommand(.25),
      ramseteCommandGenerator(drivetrain, DriverStation.getAlliance()==Alliance.Blue?new Trajectory():new Trajectory())//move to goal
        .alongWith(arm.moveToBackMiddlePostion()),
      arm.openClaw(),
      ramseteCommandGenerator(drivetrain, DriverStation.getAlliance()==Alliance.Blue?new Trajectory():new Trajectory())//drive to scale
        .alongWith(arm.moveToZeroPosition()),
      new AutoLevel(drivetrain)
    );
  }
  public static CommandBase threeGamePieces(Drivetrain drivetrain, Arm arm){
    return new SequentialCommandGroup(
      arm.moveToBackTopPosition(),
      arm.openClaw(),
      ramseteCommandGenerator(drivetrain, DriverStation.getAlliance()==Alliance.Blue?new Trajectory():new Trajectory())//drive to next game piece
        .alongWith(arm.moveToIntakePosition()),
      arm.closeClaw(),
      new WaitCommand(.25),
      ramseteCommandGenerator(drivetrain, DriverStation.getAlliance()==Alliance.Blue?new Trajectory():new Trajectory())//move to goal
        .alongWith(arm.moveToBackMiddlePostion()),
      arm.openClaw(),
      ramseteCommandGenerator(drivetrain, DriverStation.getAlliance()==Alliance.Blue?new Trajectory():new Trajectory())//move to next game piece
        .alongWith(arm.moveToIntakePosition()),
      arm.closeClaw(),
      new WaitCommand(.25),
      ramseteCommandGenerator(drivetrain, DriverStation.getAlliance()==Alliance.Blue?new Trajectory():new Trajectory())//move to goal
        .alongWith(arm.moveToBackMiddlePostion()),
      arm.openClaw(),
      ramseteCommandGenerator(drivetrain, DriverStation.getAlliance()==Alliance.Blue?new Trajectory():new Trajectory())//move to center field
    );
  } 
  public static CommandBase oneGamePieceAndLevel(Drivetrain drivetrain, Arm arm){
    return new SequentialCommandGroup(
      arm.moveToBackTopPosition(),
      arm.openClaw(),
      //move to beond the scale to taxi and then get on scale
      ramseteCommandGenerator(drivetrain, AutonPaths.getTaxiPassScale().concatenate(AutonPaths.getBackOnToScale()))
        .alongWith(arm.moveToZeroPosition()),
      new AutoLevel(drivetrain)  
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
