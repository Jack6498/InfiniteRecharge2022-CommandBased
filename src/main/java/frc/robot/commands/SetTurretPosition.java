// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.Turret;

import static frc.robot.Constants.Shooter.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetTurretPosition extends TrapezoidProfileCommand {
  /** Creates a new SetTurretPosition. */
  public SetTurretPosition(Turret subsystem) {
    super(
        // The motion profile to be executed
        new TrapezoidProfile(
            // The motion profile constraints
            new TrapezoidProfile.Constraints(turretProfileMaxVelocity, turretProfileMaxAcceleration),
            // Goal state
            new TrapezoidProfile.State(
              subsystem.getProfile().calculate(subsystem.getProfile().totalTime()).position,
              subsystem.getProfile().calculate(subsystem.getProfile().totalTime()).velocity
            ),
            // Initial state
            new TrapezoidProfile.State()),
        state -> {
          // Use current trajectory state here
        });
  }
}
