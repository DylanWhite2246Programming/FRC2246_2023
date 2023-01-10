// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.AutonConstants;
import frc.robot.subsystems.Drivetrain;

public class AutoLevel extends CommandBase {
  Drivetrain drivetrain;
  Timer timer = new Timer();
  private double error, currentAngle, drivePower;
  private final double levelValue = 2;

  /** Creates a new AutoLever. */
  public AutoLevel(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }
  
  private boolean isLevel(){return Math.abs(currentAngle)<=levelValue;}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //set cuttent angle set to the pitch of the robot
    currentAngle=drivetrain.getPitch();
    //calculate error
    error = levelValue-currentAngle;
    //calculate the amount of power need.
    drivePower=-Math.min(AutonConstants.kDriveXKp*error, 1);
    //apply power
    drivetrain.operatorDrive(()->drivePower, ()->0);
    //if angle of table is lower than 2 degrees 
    //2.5degrees is what is required to score
    if(isLevel()){timer.start();}
    else{timer.stop();timer.reset();}//if level is not level rest timer
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted){
      this.andThen(drivetrain.engageBrake());
    }else{
      this.andThen(drivetrain.STOP());
    }
    Constants.autonSuccessful = (!interrupted)&&isLevel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //when the scale is level for more than 1.5 seconds command ends
    return timer.get()>=1.5;
  }
}
