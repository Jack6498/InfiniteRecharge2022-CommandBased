// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;
import frc.surpriselib.DriveSignal;
import frc.surpriselib.MathUtils;
import io.github.oblarg.oblog.annotations.Log;

public class TurnAngle extends CommandBase {
  /** Creates a new TurnAngle. */
  DriveBase driveBase;
  @Log(name = "Target Angle")
  double gyroSetpoint;
  PIDController turnPID;
  double timeInDeadband = -1; // how long have we been in the target zone
  @Log(name = "Delta Angle")
  double error;
  Pose2d targetPose;
  public TurnAngle(DriveBase drive, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveBase = drive;
    gyroSetpoint = driveBase.getGyroAngle().getDegrees() + angle;
    turnPID = new PIDController(Constants.Drive.TurnAnglekP, Constants.Drive.TurnAnglekI, Constants.Drive.TurnAnglekD);
    turnPID.setTolerance(Constants.Drive.TurnInPlaceDeadband);
    targetPose = new Pose2d(0, 0, driveBase.getGyroAngle().plus(Rotation2d.fromDegrees(angle)));
    addRequirements(driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeInDeadband = -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // how far off are we
    error = Math.abs(gyroSetpoint - driveBase.getGyroAngle().getDegrees());
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
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.setBrakeMode(NeutralMode.Brake);
    driveBase.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timeInDeadband > 0) && (timeInDeadband + 0.08 < Timer.getFPGATimestamp()) && (error < Constants.Drive.TurnInPlaceDeadband);
  }
}
