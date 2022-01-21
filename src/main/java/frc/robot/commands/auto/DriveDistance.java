// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;
import frc.surpriselib.DriveSignal;

public class DriveDistance extends CommandBase {
  DriveBase driveBase;
  double dist;
  double speed;

  /** Creates a new DriveDistance. */
  public DriveDistance(DriveBase drive, double distanceInches, double robotSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveBase = drive;
    addRequirements(driveBase);
    dist = distanceInches;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

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
