// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class DriveArcadeOpenLoop extends CommandBase {
  /** Creates a new DriveArcade. */
  DriveBase driveBase;
  DoubleSupplier throttle, turn;
  
  public DriveArcadeOpenLoop(DriveBase drive, DoubleSupplier throttle, DoubleSupplier turn) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveBase = drive;
    // "this.thing" specifies that we mean the variable that is part of the class, not the constructor argument
    this.throttle = throttle;
    this.turn = turn;
    addRequirements(driveBase);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    driveBase.setArcadeDrive(throttle.getAsDouble(), turn.getAsDouble(), false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
