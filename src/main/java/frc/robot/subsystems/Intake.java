// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Intake.*;

public class Intake extends SubsystemBase {
  // Hardware
  VictorSPX intakeMotor;
  DoubleSolenoid leftPiston, rightPiston;

  double motorSetpoint;
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new VictorSPX(intakeVictorCANId);
    intakeMotor.configOpenloopRamp(1);
    motorSetpoint = 0.0;
    //leftPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, leftPistonForwardChannel, leftPistonReverseChannel);
    //rightPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, rightPistonForwardChannel, rightPistonReverseChannel);
  }

  public void startIntakeMotor() {
    motorSetpoint = 0.5;
  }
  public void stopIntakeMotor() {
    motorSetpoint = 0;
  }

  @Override
  public void periodic() {
    intakeMotor.set(ControlMode.PercentOutput, motorSetpoint);
  }
}
