// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

import static frc.robot.Constants.Intake.*;

public class Intake extends SubsystemBase implements Loggable {
  // Hardware
  VictorSPX frontIntakeMotor;
  VictorSPX backIntakeMotor;
  DoubleSolenoid leftPiston, rightPiston;

  double frontMotorSetpoint;
  double backMotorSetpoint;
  /** Creates a new Intake. */
  public Intake() {
    frontIntakeMotor = new VictorSPX(frontIntakeVictorCANId);
    frontIntakeMotor.configOpenloopRamp(1);
    frontMotorSetpoint = 0.0;
    backIntakeMotor = new VictorSPX(backIntakeVictorCANId);
    frontIntakeMotor.configOpenloopRamp(1);
    //leftPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, leftPistonForwardChannel, leftPistonReverseChannel);
    //rightPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, rightPistonForwardChannel, rightPistonReverseChannel);
  }

  public void startFrontIntakeMotor() {
    frontMotorSetpoint = 0.67;
  }
  public void stopFrontIntakeMotor() {
    frontMotorSetpoint = 0;
  }
  public void startBackIntakeMotor() {
    backMotorSetpoint = 0.67;
  }
public void stopBackIntakeMotor() {
  backMotorSetpoint = 0.67;
}
  @Config
  public void setMotorPercent(double percent) {
    frontMotorSetpoint = percent;
    backMotorSetpoint = percent;
  }

  @Override
  public void periodic() {
    frontIntakeMotor.set(ControlMode.PercentOutput, frontMotorSetpoint);
    backIntakeMotor.set(ControlMode.PercentOutput, backMotorSetpoint);
  }
}
