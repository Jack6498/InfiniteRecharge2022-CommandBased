// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

import static frc.robot.Constants.Intake.*;

public class Intake extends SubsystemBase implements Loggable {
  // Hardware
  VictorSPX frontIntakeMotor;
  VictorSPX backIntakeMotor;
  DoubleSolenoid frontPiston, backPiston;

  double frontMotorSetpoint;
  double backMotorSetpoint;
  /** Creates a new Intake. */
  public Intake() {
    frontIntakeMotor = new VictorSPX(frontIntakeVictorCANId);
    frontIntakeMotor.configOpenloopRamp(1);
    frontMotorSetpoint = 0.0;
    backIntakeMotor = new VictorSPX(backIntakeVictorCANId);
    frontIntakeMotor.configOpenloopRamp(1);
    frontPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, frontPistonForwardChannel, frontPistonReverseChannel);
    backPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, backPistonForwardChannel, backPistonReverseChannel);
  }
 public void lowerAndStartFrontIntake() {
    frontMotorSetpoint = 0.67;
    frontPiston.set(Value.kReverse);
 }
 public void raiseAndStopFrontIntake() {
  frontMotorSetpoint = 0;
  frontPiston.set(Value.kForward);
 }
 public void lowerAndStartBackIntake() {
  backMotorSetpoint = 0.67;
  backPiston.set(Value.kReverse);
 } 
  
public void raiseAndStopBackIntakeMotor() {
  backMotorSetpoint = 0;
  backPiston.set(Value.kForward);
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
