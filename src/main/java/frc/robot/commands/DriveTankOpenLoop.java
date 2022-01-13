// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;
import frc.robot.util.DriveSignal;

public class DriveTankOpenLoop extends CommandBase {
  /** Creates a new DriveOpenLoop. */
  DriveBase driveBase;
  DoubleSupplier leftMotor, rightMotor;
  public DriveTankOpenLoop(DriveBase drive, DoubleSupplier left, DoubleSupplier right) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveBase = drive;
    leftMotor = left;
    rightMotor = right;
    addRequirements(driveBase);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveBase.setTankDrive(new DriveSignal(leftMotor.getAsDouble(), rightMotor.getAsDouble()));
  }


}
