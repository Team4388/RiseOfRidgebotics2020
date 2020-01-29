/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  CANSparkMax m_climberMotor = new CANSparkMax(ClimberConstants.CLIMBER_SPARK_ID, MotorType.kBrushless);
  CANDigitalInput m_forwardLimit, m_reverseLimit;
  /**
   * Creates a new Climber.
   */
  public Climber() {
    m_climberMotor.restoreFactoryDefaults();
    
    m_forwardLimit = m_climberMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyClosed);
    m_reverseLimit = m_climberMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyClosed);

    m_forwardLimit.enableLimitSwitch(true);
    m_reverseLimit.enableLimitSwitch(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Runs climber motor
   * @param input the voltage to run motor at
   */
  public void runClimber(double input) {
    m_climberMotor.set(input);
  }
}
