// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//create the object linked to the LED lights -refer to the WPI lib docs
//create a series of methods/functions that allows us to switch the lights on,off and choose colours

public class Lights extends SubsystemBase {
  /** Creates a new Lights. */
  static AddressableLED LED = new AddressableLED(0);
  static AddressableLEDBuffer LED_Buffer = new AddressableLEDBuffer(30);
  static int LED_length = LED_Buffer.getLength();

  /*  colour codes rgb
  red (255, 0, 0)
  green (0, 255, 0)
  blue (0, 0, 255)
  yellow (255, 100, 0)
  purple (139, 36, 156)
  */

  public Lights() {
    LED.setLength(LED_length);
    LED.setData(LED_Buffer);
    LED.start();
  }

  public static void Set_Colour(Integer Red, Integer Green, Integer Blue) {
    for (var i = 0; i < LED_length; i++) {
      LED_Buffer.setRGB(i, Red, Green, Blue);
    }
    LED.setData(LED_Buffer);
  }

  public void purple() {
    Set_Colour(139, 36, 156);
  }

  public void yellow() {
    Set_Colour(255, 140, 0);
  }

  public static void red() {
    Set_Colour(255, 0, 0);
  }

  public static void green() {
    Set_Colour(0, 255, 0);
  }

  public void blue() {
    Set_Colour(0, 0, 255);
  }

  /*
  public void brakeMode() {
    for (var i = 30; i < LED_Buffer.getLength(); i++) {
      LED_Buffer.setRGB(i, 255, 0, 0);
    }
    LED.setData(LED_Buffer);
  }

  public void coastMode() {
    for (var i = 30; i < LED_Buffer.getLength(); i++) {
      LED_Buffer.setRGB(i, 1, 255, 0);
    }
    LED.setData(LED_Buffer);
  }
  */

  @Override
  public void periodic() {
    LED.setData(LED_Buffer);
    // This method will be called once per scheduler run
  }
}