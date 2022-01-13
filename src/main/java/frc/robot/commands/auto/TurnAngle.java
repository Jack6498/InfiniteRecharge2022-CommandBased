// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.Constants.Drive;
import frc.robot.util.DriveSignal;
import frc.robot.util.MathUtils;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;

public class TurnAngle extends CommandBase {
  /** Creates a new TurnAngle. */
  DriveBase driveBase;
  double gyroSetpoint;
  PIDController turnPID;
  double timeInDeadband = -1; // how long have we been in the target zone
  double error;
  public TurnAngle(DriveBase drive, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveBase = drive;
    gyroSetpoint = driveBase.getGyroAngle().getDegrees() + angle;
    turnPID = new PIDController(Constants.Drive.TurnAnglekP, Constants.Drive.TurnAnglekI, Constants.Drive.TurnAnglekD);
    turnPID.setTolerance(Constants.Drive.TurnInPlaceDeadband);
    addRequirements(driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log("TURNING TO ANGLE");
    timeInDeadband = -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = gyroSetpoint - driveBase.getGyroAngle().getDegrees();
    double steer = MathUtils.clamp(turnPID.calculate(error));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.setTankDrive(DriveSignal.NEUTRAL);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

  }

  private void log(String message)
  {
    System.out.print(message);
  }
}
