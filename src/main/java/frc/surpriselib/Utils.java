// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.surpriselib;

import com.ctre.phoenix.motorcontrol.NeutralMode;

/** Add your docs here. */
public class Utils {

    /**
     * @return True if <code>NeutralMode.Brake</code>, False otherwise
     */
    public static boolean neutralModeIsBrake(NeutralMode mode) {
        if (mode == NeutralMode.Brake) {
            return true;
        } else if (mode == NeutralMode.Coast) {
            return false;
        }
        return false;
    }

    public static double clamp(double value)
    {
        return clamp(value, 1.0, -1.0);
    }

    public static double clamp(double value, double max, double min)
    {
        if (value > max) 
        {
            return max;
        } 
        else if (value < min)
        {
            return min;
        }
        else
        {
            return value;
        }
    }

    public static double applyDeadband(double value, double deadband)
    {
        if (Math.abs(value) > deadband) 
        {
            if (value > 0.0) 
            {
                return (value - deadband) / (1.0 - deadband);
            }
            else
            {
                return (value + deadband) / (1.0 - deadband);
            }
        }
        else
        {
            return 0.0;
        }
    }

    public static double[] normalize(double[] values)
    { 
        double max = Math.abs(values[0]);
        boolean normFlag = max > 1; // do we normalize or just pass it back?
        for (int i = 1; i < values.length; i++) {
            if (Math.abs(values[i]) > max) { // set max to biggest magnitude motor speed
                max = Math.abs(values[i]);
                normFlag = max > 1;
            }
        }
        if (normFlag) { // normalize speeds
            for (int i = 0; i < values.length; i++) {
                values[i] /= max;
            }
        }
        return values;
    }


}
