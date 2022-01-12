// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.Constants.Drive;
import frc.robot.helpers.DriveSignal;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;

public class TurnAngle extends CommandBase {
  /** Creates a new TurnAngle. */
  DriveBase driveBase;
  private double setpoint, initialAngle, turnDegrees, currentAngle, deltaAngle;
  public TurnAngle(DriveBase drive, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveBase = drive;
    turnDegrees = angle;
    addRequirements(driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentAngle = driveBase.getGyroAngle().getDegrees();
    setpoint = initialAngle + turnDegrees;
    deltaAngle = setpoint;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = driveBase.getGyroAngle().getDegrees();
    deltaAngle = (currentAngle - setpoint) % 360;
    
    double output = deltaAngle / 360;

    if (output < 0)
    {
      output = Math.min(output, -Constants.Drive.MinAutoTurnRate);
    }
    else
    {
      output = Math.max(output, Constants.Drive.MinAutoTurnRate);
    }

    output *= Constants.Drive.NormalAutoTurnRate;
    driveBase.setTankDrive(new DriveSignal(output, -output));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.setTankDrive(DriveSignal.NEUTRAL);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(deltaAngle) < Constants.Drive.AutoTurnDeadbandDegrees;
  }
}
