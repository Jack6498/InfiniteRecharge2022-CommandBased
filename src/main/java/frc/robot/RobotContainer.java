// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.DriveArcadeOpenLoop;
import frc.robot.commands.TurretYaw;
import frc.robot.commands.auto.DriveDistanceProfiled;
import frc.robot.commands.auto.TurnAngle;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.VisionSystem;

import static frc.robot.Constants.Drive.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public DriverStation.Alliance startPosition;
  @Log
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  // controllers
  XboxController driver = new XboxController(Constants.Drive.DriverControllerId);
  // subsystems
  private final DriveBase driveBase = new DriveBase();
  private final Intake intake = new Intake();
  private final VisionSystem visionSystem = new VisionSystem();
  private final Turret turret = new Turret();
  // commands
  private final DriveArcadeOpenLoop arcadeCommand = 
    new DriveArcadeOpenLoop(
      driveBase, 
      driver::getRightTriggerAxis, 
      driver::getLeftX, 
      driver::getLeftTriggerAxis
    );
  // auto commands
  // drive x distance
  DriveDistanceProfiled driveDist = new DriveDistanceProfiled(driveBase, 2);
  // turn angle
  TurnAngle angle = new TurnAngle(driveBase, 30);
  // follow path
  // zero turret
  SequentialCommandGroup zeroTurret = new SequentialCommandGroup(
    //new ZeroTurret(turret)
  );
  private SequentialCommandGroup simplestAuto = new SequentialCommandGroup(
    new WaitCommand(1),
    new DriveDistanceProfiled(driveBase, 1),
    new TurnAngle(driveBase, 30),
    new WaitCommand(1)
  );
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Logger.configureLoggingAndConfig(this, false);
    // Configure the button bindings
    configureButtonBindings();
    driveBase.setDefaultCommand(arcadeCommand);
    turret.setDefaultCommand(new TurretYaw(turret, visionSystem));
    //turret.setDefaultCommand(zeroTurret);

    // configure autos
    autoChooser.setDefaultOption("Leave Tarmac & Stop", simplestAuto);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // button commands
    new JoystickButton(driver, Button.kRightBumper.value).whenPressed(new InstantCommand(driveBase::toggleGear, driveBase));
    //new JoystickButton(driver, Button.kLeftBumper.value).whenPressed(angle);
    new JoystickButton(driver, Button.kB.value).whenHeld(
      new PIDCommand(
        new PIDController(TurnAnglekP, TurnAnglekI, TurnAnglekD), 
        driveBase::getTurnRate, 
        0, 
        output -> driveBase.arcadeDrive(driver.getLeftTriggerAxis(), output), 
        driveBase
      )
    );
    // start/stop intake
    new JoystickButton(driver, Button.kX.value).whenHeld(new StartEndCommand(intake::startIntakeMotor, intake::stopIntakeMotor, intake));
    // set brake mode
    new JoystickButton(driver, Button.kLeftBumper.value).whenHeld(
      new StartEndCommand(
        () -> {
          driveBase.setBrakeMode(NeutralMode.Brake);
        }, 
        () -> {
          driveBase.setBrakeMode(NeutralMode.Coast);
        }, 
        driveBase
      )
    ).debounce(0.5, DebounceType.kBoth);

    new JoystickButton(driver, Button.kB.value).whenPressed(new InstantCommand(driveBase::toggleInverted, driveBase));
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
