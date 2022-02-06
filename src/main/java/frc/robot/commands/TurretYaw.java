// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.VisionSystem;
import io.github.oblarg.oblog.Loggable;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurretYaw extends CommandBase implements Loggable {
  
  Turret turret;
  VisionSystem vision;
  /** Creates a new TurretYaw. */
  public TurretYaw(Turret turret, VisionSystem vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret, vision);
    this.turret = turret;
    this.vision = vision;
  }

  @Override
  public void execute() {
    Double yaw = 0.0;
    if (vision.hasTargets()) {
      yaw = vision.getTargetData()[0];
    }
    turret.setAngleGoal(Rotation2d.fromDegrees(yaw));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
