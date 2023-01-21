// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class AlignToGamePiece extends CommandBase {
  DoubleSupplier mag;
  int piece;
  Drivetrain drivetrain;
  Vision vision;
  double error;
  PIDController turnController = new PIDController(.1, 0, 0); //TODO tune
  /**
   * @param mag forward component
   * @param piece the game piece to target 1 for cone 2 for cube
   * @param drive drive train
   */
  public AlignToGamePiece(
    DoubleSupplier mag, 
    int piece, 
    Drivetrain drive, 
    Vision vision
  ) {
    this.mag = mag;
    this.piece = piece;
    this.drivetrain = drive;
    this.vision = vision;
    addRequirements(drive, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    vision.setPipeline(piece);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(vision.getResults().hasTargets()){
      drivetrain.operatorDrive(mag, ()->turnController.calculate(vision.getResults().getBestTarget().getYaw(), 0)).schedule();
    }else{
      drivetrain.operatorDrive(mag, ()->0).schedule();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {vision.setPipeline(0);}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
