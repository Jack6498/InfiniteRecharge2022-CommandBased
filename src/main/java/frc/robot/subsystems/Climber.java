// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static frc.robot.Constants.Climber.*;


import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  TalonFX climberMotor = new TalonFX(climberMotorCANID);
  DoubleSolenoid climberRelease;
  /** Creates a new Climber. */
  public Climber() {
    new DoubleSolenoid(PneumaticsModuleType.CTREPCM, climberReleaseForwardChannel, climberReleaseReverseChannel);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
