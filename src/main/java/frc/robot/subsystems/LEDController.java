// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.surpriselib.LEDStrips;
import frc.surpriselib.Utils;

public class LEDController extends SubsystemBase {
  AddressableLED[] ledStrips = new AddressableLED[LEDStrips.values().length];
  AddressableLEDBuffer[] ledBuffers = new AddressableLEDBuffer[2];
  
  /** Creates a new LEDController. */
  public LEDController(Subsystem...subsystems) {
    Pair<AddressableLED, AddressableLEDBuffer> pair = setupStrip(1, 5);
    ledStrips[0] = pair.getFirst();
    ledBuffers[0] = pair.getSecond();

  }

  private Pair<AddressableLED, AddressableLEDBuffer> setupStrip(int port, int length) {
    AddressableLED strip = new AddressableLED(port);
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(length);
    strip.setLength(buffer.getLength());
    strip.setData(buffer);
    return new Pair<AddressableLED,AddressableLEDBuffer>(strip, buffer);
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
