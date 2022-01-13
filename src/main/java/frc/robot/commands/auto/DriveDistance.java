// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.CANMotorTools;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;
import frc.robot.util.DriveSignal;

public class DriveDistance extends CommandBase {
  DriveBase driveBase;
  double dist;
  double encoderInitialDistance;
  double encoderCurrentDistance;
  double encoderGoalDistance;
  double speed;

  /** Creates a new DriveDistance. */
  public DriveDistance(DriveBase drive, double distanceInches, double robotSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveBase = drive;
    addRequirements(driveBase);
    dist = distanceInches;
    driveBase.setSetpoint(robotSpeed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    encoderInitialDistance = driveBase.getEncoderPosition();
    encoderGoalDistance = Math.abs(dist) * Constants.Drive.EncoderTicksPerInch;
    encoderCurrentDistance = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    encoderCurrentDistance = Math.abs(driveBase.getEncoderPosition() - encoderInitialDistance);

    // if we are within 12 inches, slow down
    if (encoderCurrentDistance > (Math.abs(encoderGoalDistance) - Constants.Drive.EncoderTicksPerInch * 12)) {
      
    }

    driveBase.setTankDrive(new DriveSignal(speed, speed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
