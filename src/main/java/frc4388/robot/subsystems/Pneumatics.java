/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.PneumaticsConstants;

public class Pneumatics extends SubsystemBase {
  /* Create Solenoids */
  public DoubleSolenoid m_speedShift = new DoubleSolenoid(  PneumaticsConstants.PCM_MODULE_ID, 
                                                            PneumaticsConstants.SPEED_SHIFT_FORWARD_ID,
                                                            PneumaticsConstants.SPEED_SHIFT_REVERSE_ID );

  public DoubleSolenoid m_coolFalcon = new DoubleSolenoid(  PneumaticsConstants.PCM_MODULE_ID, 
                                                            PneumaticsConstants.COOL_FALCON_FORWARD_ID, 
                                                            PneumaticsConstants.COOL_FALCON_REVERSE_ID );

  /* Get Drive Subsystem */
  Drive m_driveSubsystem;

  public boolean m_isSpeedShiftHigh;

  /**
   * Creates a new Pneumatics.
   */
  public Pneumatics() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    runFalconCooling();
  }
  
  /**
   * Passes subsystem needed.
   * @param subsystem Subsystem needed.
   */
  public void passRequiredSubsystem(Drive subsystem) {
    m_driveSubsystem = subsystem;
  }

  /**
   * Set to high or low gear based on boolean state, true = high, false = low
   * @param state Chooses between high or low gear
   */
  public void setShiftState(boolean state) {
    if (state == true) {
			m_speedShift.set(DoubleSolenoid.Value.kReverse);
    }
		if (state == false) {
			m_speedShift.set(DoubleSolenoid.Value.kForward);
    }
    m_driveSubsystem.setRightMotorGains(state);
    m_isSpeedShiftHigh = state;
  }

  /**
   * Set to open or close solenoid that cools the falcon, true = open, false = close
   * @param state Chooses between open and close
   */
  public void coolFalcon(boolean state) {
    if (state == true) {
      m_coolFalcon.set(DoubleSolenoid.Value.kForward);
    }
    if (state == false) {
      m_coolFalcon.set(DoubleSolenoid.Value.kReverse);
    }
  }

  /**
   * Runs coolFalcon every 30 seconds for 1 second.
   */
  public void runFalconCooling() {
    if (m_driveSubsystem.m_currentTimeSec % 30 == 0) {
      coolFalcon(true);
    } else if ((m_driveSubsystem.m_currentTimeSec - 1) % 30 == 0) {
      coolFalcon(false);
    }
  }
}
