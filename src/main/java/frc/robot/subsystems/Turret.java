// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Shooter.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Turret extends SubsystemBase implements Loggable {
  
  TalonFX yawMotor = new TalonFX(yawMotorCANId);
  PIDController pid = new PIDController(turretYaw_kP, 0, turretYaw_kD);

  public Turret() {
    yawMotor.configPeakOutputForward(0.2);
    yawMotor.configPeakOutputReverse(0.2);

  }

  public boolean getForwardLimitSwitch() {
    if (yawMotor.isFwdLimitSwitchClosed() == 1) {
      return true;
    } else return false;
  }

  public PIDController getController() {
    return pid;
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

  public void setPositionGoal(int position) {

  }

  @Log
  public double getCurrentPosition() {
    return yawMotor.getSelectedSensorPosition();
  }
}
