// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Team2246.Drivestation;
import frc.robot.commands.AutoLevel;
import frc.robot.commands.MoveArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivestation drivestation = 
    new Drivestation(
      0,
      0,
     0,
     0
    );
  ShuffleboardTab mainTab = Shuffleboard.getTab("Main Tab");
  //Sendable chooser to select autonomus command
  SendableChooser<CommandBase> autonChooser = new SendableChooser<CommandBase>();
  private static final Drivetrain drivetrain = new Drivetrain();
  private static final Arm arm = new Arm();

  private static CommandBase moveArm(double value){
    return new ConditionalCommand(
      //retract arm and wait for it to reach limit
      new SequentialCommandGroup(
        arm.retractArm(),
        new WaitUntilCommand(arm::getBoomLimit).withTimeout(3),
        arm.setGoalCommand(value)
      ),
      //if collision will not happen move arm
      new MoveArm(arm, value), 
      //when the goal and curent position are on differnt sides of the robot the arm must be retracted
      ()->(Math.signum(value)!=Math.signum(arm.getMeasurement()))||value==0
    );
  }


  public CommandBase disengageBrake(){return drivetrain.disengageBrake();}
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autonChooser.addOption("Off", null);
    autonChooser.addOption("AutoLevel", new AutoLevel(drivetrain));
    mainTab.add(autonChooser).withSize(2, 1);
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //// Schedule `ExampleCommand` when `exampleCondition` changes to `true` //TODO take a look
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return selected auton to run
    return autonChooser.getSelected();
  }
}
