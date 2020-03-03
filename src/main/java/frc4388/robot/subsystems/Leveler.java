/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.LevelerConstants;

public class Leveler extends SubsystemBase {
  CANSparkMax m_levelerMotor = new CANSparkMax(LevelerConstants.LEVELER_CAN_ID, MotorType.kBrushless);

  private final Climber m_robotClimber = new Climber();

  /**
   * Creates a new Leveler.
   */
  public Leveler() {
    m_levelerMotor.restoreFactoryDefaults();

    m_levelerMotor.setIdleMode(IdleMode.kBrake);
    m_levelerMotor.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Runs intake motor
   * @param input the percent output to run motor at
   */
  public void runLeveler(double input) {
    if(m_robotClimber.climberSafety){
      m_levelerMotor.set(input);
    }
    else{
      m_levelerMotor.set(0);
    }
  }
}
