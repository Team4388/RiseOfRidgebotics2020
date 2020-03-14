/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.utility.controller;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc4388.robot.RobotContainer;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

/**
 * A {@link Button} that gets its state from a {@link GenericHID}.
 */
public class JoystickManualButton extends Button {
    private final GenericHID m_joystick;
    private final int m_buttonNumber;
    private boolean m_buttonType;

    /**
     * Creates a joystick button for triggering commands.
     *
     * @param joystick     The GenericHID object that has the button (e.g. Joystick,
     *                     KinectStick, etc)
     * @param buttonNumber The button number (see
     *                     {@link GenericHID#getRawButton(int) }
     */
    public JoystickManualButton(GenericHID joystick, int buttonNumber, boolean buttonType) {
    requireNonNullParam(joystick, "joystick", "JoystickButton");

    m_joystick = joystick;
    m_buttonNumber = buttonNumber;
    m_buttonType = buttonType;
  }

  /**
   * Gets the value of the joystick button.
   *
   * @return The value of the joystick button
   */
  @Override
  public boolean get() {
    boolean m = RobotContainer.m_isShooterManual;
    if (m_buttonType == m) {
        return m_joystick.getRawButton(m_buttonNumber);
    } else {
        return false;
    }
  }
}
