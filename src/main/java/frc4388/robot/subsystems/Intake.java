/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.IntakeConstants;



public class Intake extends SubsystemBase {
  CANSparkMax m_intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_SPARK_ID, MotorType.kBrushless);
  CANSparkMax m_extenderMotor = new CANSparkMax(IntakeConstants.EXTENDER_SPARK_ID, MotorType.kBrushless);
  CANDigitalInput m_extenderForwardLimit;
  CANDigitalInput m_extenderReverseLimit;
  public boolean isExtended = false;

  /**
   * Creates a new Intake.
   */
  public Intake() {
    

    m_intakeMotor.restoreFactoryDefaults();
    m_extenderMotor.restoreFactoryDefaults();

    m_intakeMotor.setIdleMode(IdleMode.kCoast);
    m_extenderMotor.setIdleMode(IdleMode.kBrake);
    m_intakeMotor.setInverted(false);
    m_extenderMotor.setInverted(true);
    
    m_extenderForwardLimit = m_extenderMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    m_extenderReverseLimit = m_extenderMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyClosed);
    m_extenderForwardLimit.enableLimitSwitch(true);
    m_extenderReverseLimit.enableLimitSwitch(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Runs intake motor
   * @param input the percent output to run motor at
   */
  public void runIntake(double input) {
    m_intakeMotor.set(input); //this is the speed changer
  }

  public void runExtender(double input){
    m_extenderMotor.set(input);
  }
   /**
   * Runs extender motor
   * @param input the percent output to run motor at
   */
  /*public void runExtender(double input) {
    if (m_extenderForwardLimit.get()) {
      isExtended = true;
    }
    else if (m_extenderReverseLimit.get()) {
      isExtended = false;
    }
    else{
      m_extenderMotor.set(-input);
    }
    
    if (isExtended == false) {
      m_extenderMotor.set(input);
    }
    if (isExtended == true) {
      m_extenderMotor.set(-input);
    }
  }*/
}