// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;
import frc.surpriselib.DriveSignal;
import frc.surpriselib.MathUtils;

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
    // how far off are we
    error = gyroSetpoint - driveBase.getGyroAngle().getDegrees();
    // get steering command
    double steer = MathUtils.clamp(turnPID.calculate(error, 0));
    // send steering command
    driveBase.setTankDrive(new DriveSignal(-steer, steer));
    // are we inside deadband
    if (error < Constants.Drive.TurnInPlaceDeadband) {
      if (timeInDeadband < 0) {
        timeInDeadband = Timer.getFPGATimestamp();
      }
    }
    else
    {
      timeInDeadband = -1;
    }
    SmartDashboard.putNumber("Gyro Angle", driveBase.getGyroAngle().getDegrees());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    log("TURN ENDED");
    driveBase.setBrakeMode(NeutralMode.Brake);
    driveBase.setTankDrive(DriveSignal.NEUTRAL);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timeInDeadband > 0) && (timeInDeadband + 0.08 < Timer.getFPGATimestamp()) && (error < Constants.Drive.TurnInPlaceDeadband);
  }

  private void log(String message)
  {
    System.out.print(message);
  }
}
