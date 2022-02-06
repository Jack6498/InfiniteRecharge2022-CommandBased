// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Shooter.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Turret extends SubsystemBase implements Loggable {
  
  TalonFX yawMotor = new TalonFX(yawMotorCANId);
  Rotation2d angleGoal = Rotation2d.fromDegrees(0);
  double rotationError = 0.0;

  public Turret() {
    yawMotor.configPeakOutputForward(0.2);
    yawMotor.configPeakOutputReverse(0.2);
    yawMotor.config_kP(0, turretYaw_kP);
    yawMotor.config_kI(0, 0);
    yawMotor.config_kD(0, turretYaw_kD);

  }

  public boolean getForwardLimitSwitch() {
    if (yawMotor.isFwdLimitSwitchClosed() == 1) {
      return true;
    } else return false;
  }

  public boolean getReverseLimitSwitch() {
    if (yawMotor.isRevLimitSwitchClosed() == 1) {
      return true;
    } else return false;
  }

  public void resetSensors() {
    yawMotor.setSelectedSensorPosition(0);
  }

  public void openLoop(double demand) {
    yawMotor.set(ControlMode.PercentOutput, demand);
  }

  public void setAngleGoal(Rotation2d angle) {
    yawMotor.set(ControlMode.Position, angle.getRadians() / (2 * Math.PI / turretTicksPerRotation));
  }

  @Log(name = "Current Turret Yaw")
  public double getCurrentPosition() {
    return yawMotor.getSelectedSensorPosition();
  }

  @Log(name = "Position Setpoint")
  public double getPositionSetpoint() {
    return yawMotor.getClosedLoopTarget();
  }

  @Log(name = "Position Error")
  public double getPositionError() {
    return yawMotor.getClosedLoopError();
  }

  public void reset(Rotation2d angle) {
    yawMotor.setSelectedSensorPosition((int)(angle.getRadians() / (2 * Math.PI / turretTicksPerRotation)));
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(Units.radiansToDegrees(yawMotor.getSelectedSensorPosition() / turretTicksPerRotation * 2 * Math.PI));
  }
}
