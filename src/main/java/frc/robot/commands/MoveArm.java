// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RobotConstruction;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase {
  Arm arm; double position;
  /** Creates a new MoveArm. */
  public MoveArm(Arm arm, double position) {
    this.arm = arm;
    this.position = position;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(arm.getMeasurement()<RobotConstruction.kArmEncoderOffset+Math.toRadians(5)){
      arm.disable();
    }else{
      arm.setGoal(position);
      arm.enable();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(arm.getController().getGoal().position==RobotConstruction.kArmEncoderOffset){
      arm.disable();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.atGoal();
  }
}
