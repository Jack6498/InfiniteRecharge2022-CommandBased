// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import static frc.robot.Constants.Drive.maxPathAcceleration;
import static frc.robot.Constants.Drive.maxPathVelocity;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.DriveBase;

public class DriveDistanceProfiled extends TrapezoidProfileCommand {
  DriveBase driveBase;
  double dist;
  double speed;

  /** Creates a new DriveDistance. */
  public DriveDistanceProfiled(DriveBase drive, double distanceMeters) {
    super(
      new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
          maxPathVelocity, 
          maxPathAcceleration
        ), 
        // Trajectory implicitly starts at p=0,v=0
        // we specify the endpoint (waypoint 1) as p=distance,v=0 (stop at trajectory end)
        new TrapezoidProfile.State(distanceMeters, 0)
      ), 
      profilePresentState -> drive.consumeTrapezoidState(profilePresentState, profilePresentState), 
      drive
    );
    drive.resetSensors();
  }
}
