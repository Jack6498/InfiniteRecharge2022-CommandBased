// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.DriveArcadeOpenLoop;
import frc.robot.commands.auto.TurnAngle;
import frc.robot.subsystems.DriveBase;
import io.github.oblarg.oblog.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // controllers
  XboxController driver = new XboxController(Constants.Drive.DriverControllerId);
  // subsystems
  private final DriveBase driveBase = new DriveBase();
  // commands
  private final DriveArcadeOpenLoop arcadeCommand = 
    new DriveArcadeOpenLoop(
      driveBase, 
      driver::getLeftTriggerAxis, 
      driver::getRightX, 
      driver::getRightTriggerAxis
    );
  // auto commands
  // drive x distance
  // turn angle
  TurnAngle angle = new TurnAngle(driveBase, 30);
  // follow path
  public DriverStation.Alliance startPosition;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Logger.configureLoggingAndConfig(this, false);
    // Configure the button bindings
    configureButtonBindings();
    driveBase.setDefaultCommand(arcadeCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // button commands
    new JoystickButton(driver, Button.kA.value).whenPressed(new InstantCommand(driveBase::toggleGear, driveBase));
    new JoystickButton(driver, Button.kLeftBumper.value).whenPressed(angle);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new TurnAngle(driveBase, 0);
  }
}
