// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.surpriselib;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.Constants;

/** Add your docs here. */
public class DriveSignal {
    public double leftMotor;
    public double rightMotor;
    public NeutralMode brakeMode;

    public DriveSignal(double left, double right, NeutralMode brakeMode)
    {
        this.leftMotor = left;
        this.rightMotor = right;
        this.brakeMode = brakeMode;
    }

    public DriveSignal(double left, double right)
    {
        this(left, right, Constants.Drive.BrakeModeDefault);
    }

    public static DriveSignal NEUTRAL = 
    new DriveSignal(0, 0, NeutralMode.Coast);
    public static DriveSignal BRAKE = 
    new DriveSignal(0, 0, NeutralMode.Brake);

    @Override
    public String toString()
    {
        return String.format("L: %.2f, R: %.2f, B: %s", leftMotor, rightMotor, brakeMode);
    }

}
