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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  CANSparkMax m_climberMotor = new CANSparkMax(ClimberConstants.CLIMBER_SPARK_ID, MotorType.kBrushless);
  CANDigitalInput m_climberForwardLimit, m_climberReverseLimit;

  Servo m_servo;
  //Spark m_spark = new Spark(4);

  public boolean climberSafety = false;

  /**
   * Creates a new Climber.
   */
  public Climber() {
    m_servo = new Servo(4);

    m_climberMotor.restoreFactoryDefaults();

    m_climberMotor.setIdleMode(IdleMode.kBrake);
    m_climberMotor.setInverted(true);
    
    m_climberForwardLimit = m_climberMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    m_climberReverseLimit = m_climberMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    m_climberForwardLimit.enableLimitSwitch(true);
    m_climberReverseLimit.enableLimitSwitch(true);
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
    if(climberSafety){
      input *= 1.0;
      m_climberMotor.set(input);
    }
    else{
      m_climberMotor.set(0);
    }
  }

  /* Safety Button for Climber */
  public void setSafetyPressed()
  {
    climberSafety = true;
  }

  /* Safety Button for Climber set back to false */
  public void setSafetyNotPressed()
  {
    climberSafety = false;
  }

  /**
   * @param shift true to enage rachet, false to disengage
   */
  public void shiftServo(boolean shift) {
    if (shift) {
      m_servo.setPosition(0.5);
    } else {
      m_servo.setPosition(0.56);
    }
  }
}
