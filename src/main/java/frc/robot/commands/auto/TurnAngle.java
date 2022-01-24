// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import static frc.robot.Constants.Drive.TurnAnglekD;
import static frc.robot.Constants.Drive.TurnAnglekI;
import static frc.robot.Constants.Drive.TurnAnglekP;
import static frc.robot.Constants.Drive.TurnInPlaceDeadband;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class TurnAngle extends PIDCommand implements Loggable {
  private final DriveBase driveBase;
  @Config
  private final PIDController pid;

  public TurnAngle(DriveBase drive, double angle) {
    super(
      new PIDController(TurnAnglekP, TurnAnglekI, TurnAnglekD), 
      drive::getHeading, 
      angle, 
      output -> {
        if (output > 0) { // turn clockwise
          drive.arcadeDrive(0, output);
        } else if (output < 0) { // turn counterclockwise
          drive.arcadeDrive(0, -output);
        }
      },
      drive);
      driveBase = drive;
      pid = getController();
      // working with angles so we want to wrap
      pid.enableContinuousInput(-180, 180);
      pid.setTolerance(TurnInPlaceDeadband, 5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.setBrakeMode(NeutralMode.Brake);
    driveBase.arcadeDrive(0, 0);
  }

  @Log
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }

  @Log
  public double getAngularError() {
    return pid.getPositionError();
  }
}
