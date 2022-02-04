// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.surpriselib.LEDStrips;
import frc.surpriselib.Utils;

public class LEDController extends SubsystemBase {
  AddressableLED driveStatusLED;
  AddressableLEDBuffer driveStatusBuffer;
  HashMap<String,Color> colors = new HashMap<>();
  
  /** Creates a new LEDController. */
  public LEDController(Subsystem...subsystems) {
    // colors
    colors.put("red", Color.kRed);
    colors.put("green", Color.kGreen);
    colors.put("blue", Color.kBlue);
    colors.put("orange", Color.kOrange);
    colors.put("off", Color.kBlack);

    driveStatusLED = new AddressableLED(1);
    driveStatusBuffer  = new AddressableLEDBuffer(5);
    driveStatusLED.setLength(driveStatusBuffer.getLength());

    driveStatusLED.setData(driveStatusBuffer);
  }

  public void setStripSolid(LEDStrips strip, Color ledColor) {
    setStripSolid(LEDStrips.GEAR, ledColor, 1);
  }

  public void setStripSolid(LEDStrips strip, Color ledColor, double reduction) {
    reduction = Utils.clamp(reduction, 1, 0);
    Color adjustedColor = new Color(
      ledColor.red * reduction, 
      ledColor.green * reduction, 
      ledColor.blue * reduction
    );

    for (int i = 0; i < driveStatusBuffer.getLength(); i++) {
      driveStatusBuffer.setLED(i, adjustedColor);
    }
  }
}
