/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc4388.robot.Constants.LEDConstants;
import frc4388.utility.LEDPatterns;

public class LED extends SubsystemBase {

  public static float currentLED;
  public static float defaultLED;
  public static Spark LEDController;

  /**
   * Creates a new LED to run a 5v LED Strip using a Rev 
   * Robotics Blinkin LED Driver
   */
  public LED(){
    LEDController = new Spark(LEDConstants.LED_SPARK_ID);
    defaultLED = LEDConstants.DEFAULT_PATTERN.getValue();
    runDefaultLED();
    LEDController.set(currentLED);
    System.err.println("In the Beginning, there was Joe.\nAnd he said, 'Let there be LEDs.'\nAnd it was good.");
  }

  /**
   * Sends an update to the LED Driver with the current LED value.
   * This method should be run continously to keep the lights on.
   */
  public void updateLED(){
    LEDController.set(currentLED);
  }

  /**
   * 
   */
  public void runDefaultLED() {
    setPattern(defaultLED);
  }

  /**
   * Changes the default LED by an amount
   * @param amount the amount to increment led by
   */
  public void incrementLED(float amount) {
    defaultLED += amount;
    if (defaultLED > 1) {
      defaultLED -= 2;
    }
    if (defaultLED < -1) {
      defaultLED += 2;
    }
  }

  /**
   * Sets the current LED pattern. This method should only be run
   * whenever you want to change the current LED.
   * @param pattern LEDPattern to set the Blinkin to.
   */
  public void setPattern(float pattern){
    currentLED = pattern;
    LEDController.set(pattern);
  }

  /**
   * Sets the current LED pattern. This method should only be run
   * whenever you want to change the current LED.
   * @param pattern LEDPattern to set the Blinkin to.
   */
  public void setPattern(LEDPatterns pattern){
    currentLED = pattern.getValue();
    LEDController.set(currentLED);
  }

  @Override
  public void periodic(){
    //SmartDashboard.putNumber("LED", currentLED);
  }
}