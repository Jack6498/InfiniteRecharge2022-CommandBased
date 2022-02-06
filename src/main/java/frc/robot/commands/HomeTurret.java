// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class HomeTurret extends CommandBase {
  Turret turret;
  boolean homed;
  boolean center;
  /** Creates a new HomeTurret. */
  public HomeTurret(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    homed = false;
    center = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!homed) {
      turret.openLoop(-0.12);
      if (turret.getForwardLimitSwitch()) {
        turret.reset(Rotation2d.fromDegrees(135+13.35));   
        homed = true;   
      }
      if (turret.getReverseLimitSwitch()) {
        turret.reset(Rotation2d.fromDegrees(-135-13.25));
        homed = true;
      }
    } else {
      if (!center) {
        turret.setAngleGoal(Rotation2d.fromDegrees(0));
        if (Math.abs(turret.getAngle().getDegrees()) < 0.5) {
          center = true;
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return center;
  }
}
