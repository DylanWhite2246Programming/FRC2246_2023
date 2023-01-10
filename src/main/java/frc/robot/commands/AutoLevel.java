// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;

public class AutoLevel extends CommandBase {
  Drivetrain drivetrain;
  Timer timer = new Timer();

  /** Creates a new AutoLever. */
  public AutoLevel(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted&&DriverStation.isAutonomous()){
      this.andThen(
        drivetrain.STOP(),
        drivetrain.engageBrake(),
        new WaitCommand(3),
        drivetrain.disengageBrake()
      );
    }else if(!interrupted){
      this.andThen(
        drivetrain.STOP(),
        drivetrain.engageBrake()
      );
    }else{
      this.andThen(drivetrain.STOP());
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
