// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.surpriselib.MathUtils;

public class OpenLoopTurret extends CommandBase {
  Turret turret;
  DoubleSupplier demander;
  /** Creates a new OpenLoopTurret. */
  public OpenLoopTurret(DoubleSupplier demander, Turret turret) {
    this.turret = turret;
    this.demander = demander;
    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    turret.openLoop(MathUtils.clamp(demander.getAsDouble(), 0.2, -0.2));
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
