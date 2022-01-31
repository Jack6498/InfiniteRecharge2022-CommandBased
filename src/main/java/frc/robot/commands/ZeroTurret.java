// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class ZeroTurret extends CommandBase {
  Turret turret;
  boolean fwdLimit, revLimit;
  double midPoint;
  TrapezoidProfile center;
  public ZeroTurret(Turret subsystem) {
    turret = subsystem;
    addRequirements(turret);
    fwdLimit = false;
    revLimit = false;
    midPoint = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!revLimit) {
      // havent checked rev limit yet
      turret.openLoop(-0.05);
      if (turret.getReverseLimitSwitch()) {
        revLimit = true;
        turret.resetSensors();
      }
    } else if (!fwdLimit) {
      // havent checked fwd limit yet
      turret.openLoop(0.05);
      if (turret.getForwardLimitSwitch()) {
        fwdLimit = true;
        midPoint = turret.getCurrentPosition() / 2;
        center = new TrapezoidProfile(new TrapezoidProfile.Constraints(1, 1), new TrapezoidProfile.State(midPoint, 0));
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setProfile(center);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return fwdLimit && revLimit;
  }
}
