// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Drivetrain;

public class AlignToGoal extends CommandBase {
  Drivetrain drivetrain;
  int goalType, row; DoubleSupplier mag;
  Translation2d choosenGoalPose;
  PIDController turnController = new PIDController(.1, 0, 0); //TODO tune
  /** Creates a new AlignToGoal. */
  public AlignToGoal(
    Drivetrain drive, 
    int goalType, 
    int row,
    DoubleSupplier mag
  ) {
    this.drivetrain = drive;
    this.mag = mag;
    this.goalType = goalType;
    this.row = row;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(goalType == 0){
      choosenGoalPose = FieldConstants.choosePeg(row, drivetrain.getPose2d());
    }else if(goalType == 1){
      choosenGoalPose = FieldConstants.chooseShelf(row, drivetrain.getPose2d());
    }else{
      choosenGoalPose = new Translation2d();
      System.out.println("you werent suposed to get here");
    }
    drivetrain.operatorDrive(
      mag, 
      ()->{
        if((DriverStation.getAlliance()==Alliance.Red)||Math.abs(drivetrain.getRotation2d().getRadians()%(Math.PI*2))>(Math.PI/2)){
          return turnController.calculate(
            drivetrain.getRotation2d().getRadians()%(Math.PI*2), 
            new Pose2d(choosenGoalPose, new Rotation2d()).relativeTo(drivetrain.getPose2d()).getRotation().getRadians()+Math.PI
          );
        }else{
          return turnController.calculate(
            drivetrain.getRotation2d().getRadians()%(Math.PI*2), 
            new Pose2d(choosenGoalPose, new Rotation2d()).relativeTo(drivetrain.getPose2d()).getRotation().getRadians()
          );
        }
      }
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
