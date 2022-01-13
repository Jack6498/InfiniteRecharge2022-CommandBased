// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.Constants;

/**
 * Offers various angular<->angular and angular<->linear unit conversions
*/
public class UnitConverter {
    
    private static double rotationsToInches(final double rotations) {
        return rotations * (Constants.Drive.WheelDiameterInches * Math.PI);
    }

    private static double rpmToInchesPerSecond(final double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(final double inches) {
        return inches / (Constants.Drive.WheelDiameterInches * Math.PI);
    }

    private static double inchesPerSecondToRpm(final double inchesPerSecond) {
        return inchesToRotations(inchesPerSecond) * 60;
    }
}
